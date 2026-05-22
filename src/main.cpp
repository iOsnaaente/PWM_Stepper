/**
 * @file    main.cpp
 * @author  Bruno Gabriel Flores Sampaio
 * @author  Icaro Marques de Campos
 * @date    13 de Junho de 2025
 *
 * @brief   Firmware principal do VSSS Stepper.
 * @details Usa a arquitetura de comunicacao portada do V3S_IDF_32_Firmware:
 *              - interfaces/comm_base.h      (CommBase)
 *              - modules/wifi/wifi_comm.h    (WiFiComm UDP via ESP-IDF)
 *              - controllers/communication/  (ProtocolComm: pack/unpack + ACK/REQ_RESP + event loop)
 *          O handler externo envia PROTO_CMD_SET_TARGET_VEL com dois floats
 *          (left, right) em [-1.0, 1.0] que sao aplicados aos steppers.
 */

#include "../board_config.h"

#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Display/debug_display.h"

#include "interfaces/comm_base.h"
#include "interfaces/proto_base.h"
#include "controllers/communication/communication.h"

#ifdef USE_ESPNOW_TRANSPORT
  #include "modules/espnow/espnow_comm.h"
  #ifndef ESPNOW_CHANNEL
    #define ESPNOW_CHANNEL 6
  #endif
#else
  #include "modules/wifi/wifi_comm.h"
  #ifndef WIFI_SSID
  #error "WIFI_SSID nao definido. Copie include/credentials.template.h para include/credentials.h."
  #endif
  #ifndef WIFI_PASSWORD
  #error "WIFI_PASSWORD nao definido."
  #endif
  #ifndef UDP_LISTEN_PORT
  #error "UDP_LISTEN_PORT nao definido."
  #endif
#endif

#ifndef ID_DEVICE
#define ID_DEVICE 0x01
#endif

// ------------------------------------------------------------------
// Hardware / robot
// ------------------------------------------------------------------
Stepper *motor_esquerdo = nullptr;
Stepper *motor_direito  = nullptr;
Robot   *robot          = nullptr;

// ------------------------------------------------------------------
// Communication stack
// ------------------------------------------------------------------
#ifdef USE_ESPNOW_TRANSPORT
static espnow_comm_config_t   espnow_config = {};
static ESPNowComm            *espnow_comm   = nullptr;
#else
static wifi_comm_config_t     wifi_config   = {};
static WiFiComm              *wifi_comm     = nullptr;
#endif
static ProtocolComm          *comm          = nullptr;

// ------------------------------------------------------------------
// Watchdog: stop motors if no command arrives within timeout
// ------------------------------------------------------------------
static const uint32_t COMMAND_TIMEOUT_MS = 500;
static TickType_t     last_command_tick  = 0;
static const float    CMD_DEADZONE       = 0.05f;
static const uint32_t WIFI_START_DELAY_MS = 3000;

static inline float clamp_unit(float v) { return fminf(fmaxf(v, -1.0f), 1.0f); }

static void apply_wheel_command(float left, float right) {
    float lcmd = (fabsf(left)  < CMD_DEADZONE) ? 0.0f : clamp_unit(left);
    float rcmd = (fabsf(right) < CMD_DEADZONE) ? 0.0f : clamp_unit(right);

    if (lcmd == 0.0f && rcmd == 0.0f) {
        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);
    } else {
        if (!robot->get_torque()) robot->set_torque(true);
        robot->drive_wheels(lcmd, rcmd);
    }
    last_command_tick = xTaskGetTickCount();
}

static void control_watchdog_step() {
    if (last_command_tick != 0) {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_command_tick) * portTICK_PERIOD_MS > COMMAND_TIMEOUT_MS) {
            robot->drive_wheels(0.0f, 0.0f);
            robot->set_torque(false);
            last_command_tick = 0;
            DEBUG_SERIAL("CTRL", "Timeout stop");
        }
    }
}

// ------------------------------------------------------------------
// Protocol callbacks
// ------------------------------------------------------------------
static void handle_set_target_vel(
    void *handler_arg, esp_event_base_t base, int32_t id, void *event_data
) {
    (void)handler_arg; (void)base; (void)id;
    ProtocolParsed_t *msg = *(ProtocolParsed_t**)event_data;
    if (!msg) return;

    if (msg->packet.header.payload_len != sizeof(float) * 2) {
        DEBUG_SERIAL("CTRL", "SET_TARGET_VEL payload len invalido: %u",
            (unsigned)msg->packet.header.payload_len);
        return;
    }

    float left = 0.0f, right = 0.0f;
    memcpy(&left,  msg->packet.payload,                sizeof(float));
    memcpy(&right, msg->packet.payload + sizeof(float), sizeof(float));

    apply_wheel_command(left, right);

    // Throttled CTRL log (~4 Hz) so OLED + serial don't drown at command rates.
    static uint32_t last_ctrl_log_ms = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_ctrl_log_ms >= 250) {
        DEBUG_SERIAL("CTRL", "L=%.2f R=%.2f", left, right);
        last_ctrl_log_ms = now_ms;
    }
}

static void handle_stop(
    void *handler_arg, esp_event_base_t base, int32_t id, void *event_data
) {
    (void)handler_arg; (void)base; (void)id; (void)event_data;
    robot->drive_wheels(0.0f, 0.0f);
    robot->set_torque(false);
    last_command_tick = 0;
    DEBUG_SERIAL("CTRL", "STOP");
}

static void handle_ping(
    void *handler_arg, esp_event_base_t base, int32_t id, void *event_data
) {
    (void)handler_arg; (void)base; (void)id;
    ProtocolParsed_t *msg = *(ProtocolParsed_t**)event_data;
    if (!msg) return;
    if (msg->packet.header.flags & PROTO_FLAG_REQ_RESP) {
        comm->send_reply(msg, nullptr, 0);
    }
}

// ------------------------------------------------------------------
// FreeRTOS tasks
// ------------------------------------------------------------------
void motor_update_Task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        motor_esquerdo->update(0.010f);
        motor_direito->update(0.010f);
        control_watchdog_step();
        vTaskDelayUntil(&last, period);
    }
}

#ifndef USE_ESPNOW_TRANSPORT
// Periodic WiFi watcher: logs state changes, refreshes IP shown on
// serial + OLED, runs a periodic scan when down, and drives a backoff
// reconnect cycle when the event-handler retry budget is exhausted.
void wifi_monitor_Task(void *pvParameters) {
    (void)pvParameters;
    bool     last_connected      = false;
    uint32_t last_log_ms         = 0;
    uint32_t last_scan_ms        = 0;
    uint32_t disconnected_since  = 0;
    uint32_t backoff_ms          = 2000;   // grows up to BACKOFF_MAX_MS
    uint32_t last_reconnect_ms   = 0;
    const uint32_t BACKOFF_MAX_MS = 30000;
    const TickType_t period      = pdMS_TO_TICKS(1000);

    for (;;) {
        bool now_connected = wifi_comm && wifi_comm->connected;
        uint32_t now_ms = millis();

        if (now_connected != last_connected) {
            if (now_connected) {
                wifi_ip4_t addr;
                char ip_str[16] = "?";
                if (wifi_comm->get_local_address(&addr) == COMM_RET_OK) {
                    snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
                        addr.ip[0], addr.ip[1], addr.ip[2], addr.ip[3]);
                }
                DEBUG_SERIAL("WIFI", "STATE CHANGE: connected, IP=%s, RSSI=%d",
                    ip_str, (int)wifi_comm->get_rssi());
                debug_display_set_ip(ip_str);
                backoff_ms = 2000;          // fresh start
                disconnected_since = 0;
            } else {
                DEBUG_SERIAL("WIFI", "STATE CHANGE: disconnected");
                debug_display_set_ip("no wifi");
                disconnected_since = now_ms;
                last_reconnect_ms  = now_ms;
            }
            last_connected = now_connected;
        }

        // Heartbeat / status log.
        if (!now_connected && (now_ms - last_log_ms) >= 5000) {
            DEBUG_SERIAL("WIFI", "down %us backoff=%ums",
                (unsigned)((now_ms - disconnected_since) / 1000),
                (unsigned)backoff_ms);
            last_log_ms = now_ms;
        } else if (now_connected && (now_ms - last_log_ms) >= 10000) {
            DEBUG_SERIAL("WIFI", "ok IP=%u.%u.%u.%u RSSI=%d",
                wifi_comm->ip.ip[0], wifi_comm->ip.ip[1],
                wifi_comm->ip.ip[2], wifi_comm->ip.ip[3],
                (int)wifi_comm->get_rssi());
            last_log_ms = now_ms;
        }

        // Backoff-paced reconnect once the event handler stopped trying.
        if (!now_connected &&
            (now_ms - last_reconnect_ms) >= backoff_ms)
        {
            DEBUG_SERIAL("WIFI", "Reconnect (backoff %ums)",
                (unsigned)backoff_ms);
            esp_wifi_disconnect();
            esp_wifi_connect();
            last_reconnect_ms = now_ms;
            backoff_ms = (backoff_ms * 2 < BACKOFF_MAX_MS)
                       ? backoff_ms * 2
                       : BACKOFF_MAX_MS;
        }

        // Periodic scan every 20s while disconnected, for diagnostics.
        if (!now_connected && (now_ms - last_scan_ms) >= 20000) {
            last_scan_ms = now_ms;
            wifi_ap_record_t ap = {};
            bool found = wifi_scan_once(WIFI_SSID, &ap);
            if (found) {
                DEBUG_SERIAL("SCAN",
                    "AP visivel RSSI=%d ch=%d auth=%d",
                    (int)ap.rssi, (int)ap.primary, (int)ap.authmode);
            } else {
                DEBUG_SERIAL("SCAN", "AP \"%s\" NAO visivel", WIFI_SSID);
            }
        }

        vTaskDelay(period);
    }
}
#endif  // !USE_ESPNOW_TRANSPORT

// ------------------------------------------------------------------
// Arduino setup / loop. Guarded against the self-test build flags so
// only one setup() is compiled at a time.
// ------------------------------------------------------------------
#if !defined(MOTOR_SELF_TEST) && !defined(USE_ESPNOW_CONTROL) \
    && !defined(GY521_SELF_TEST) && !defined(KINEMATICS_SELF_TEST) \
    && !defined(STRAIGHT_SELF_TEST)

// Force A4988 EN HIGH (disabled) and STEP/DIR LOW the instant setup() runs,
// before any other init can stall. A4988 EN has an internal pull-DOWN, so a
// floating EN reads LOW and enables the H-bridges. Energized coils during the
// USB-CDC wait + OLED init cause a 3.3V sag that triggers brownout reset.
static void motors_safe_init(void) {
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << ENABLE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&en_cfg);
    gpio_set_level(ENABLE_PIN, 1);  // A4988 EN active-low: HIGH = disabled

    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << M1_VEL_PIN) | (1ULL << M2_VEL_PIN)
                      | (1ULL << M1_DIR_PIN) | (1ULL << M2_DIR_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&step_cfg);
    gpio_set_level(M1_VEL_PIN, 0);
    gpio_set_level(M2_VEL_PIN, 0);
    gpio_set_level(M1_DIR_PIN, 0);
    gpio_set_level(M2_DIR_PIN, 0);
}

void setup() {
    motors_safe_init();
    serial_debugger_init();
    DEBUG_SERIAL("RESET", "Reset reason: %d", (int)esp_reset_reason());
    DEBUG_SERIAL("SERIAL INIT", "Baudrate: %d", USB_BUS_BAUDRATE);

    debug_display_init();
    DEBUG_SERIAL("OLED", "Display debug inicializado");

    // Reset reason logged AFTER OLED init so the message survives USB-CDC
    // re-enumeration (the line printed earlier is often lost on reboot loops).
    int rr = (int)esp_reset_reason();
    DEBUG_SERIAL("BOOT", "ResetReason=%d (1=POR 3=SW 8=WDT 11=PANIC 14=BROWNOUT)", rr);

    // Granular logs around motor construction so we know which step crashes.
    DEBUG_SERIAL("BOOT", "Stepper L ctor begin");
    motor_esquerdo = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                                 LEDC_CHANNEL_0, LEDC_TIMER_0,
                                 Stepper::DRIVER_A4988, 1, false);
    DEBUG_SERIAL("BOOT", "Stepper L ctor done");

    DEBUG_SERIAL("BOOT", "Stepper R ctor begin");
    motor_direito  = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                                 LEDC_CHANNEL_1, LEDC_TIMER_1,
                                 Stepper::DRIVER_A4988, 1, true);
    DEBUG_SERIAL("BOOT", "Stepper R ctor done");

    robot = new Robot(*motor_esquerdo, *motor_direito);
    DEBUG_SERIAL("BOOT", "Robot ctor done");

    robot->stop();
    DEBUG_SERIAL("BOOT", "Robot stop done");

    DEBUG_SERIAL("POWER", "Aguardando %lu ms antes de iniciar comm",
        (unsigned long)WIFI_START_DELAY_MS);
    delay(WIFI_START_DELAY_MS);

#ifdef USE_ESPNOW_TRANSPORT
    // ---- ProtocolComm + ESPNowComm bring-up (broadcast, fixed channel) ----
    espnow_config.channel    = (uint8_t)ESPNOW_CHANNEL;
    static const uint8_t broadcast_mac[6] = ESPNOW_BROADCAST_MAC;
    memcpy(espnow_config.peer_mac, broadcast_mac, 6);
    espnow_config.auto_start = true;

    espnow_comm = new ESPNowComm("ESPNow", &espnow_config);
    comm        = new ProtocolComm("Communication", ID_DEVICE, espnow_comm);
#else
    // ---- ProtocolComm + WiFiComm bring-up ----
    wifi_config.mode = WIFI_MODE_STA;
    wifi_config.sta.ssid     = WIFI_SSID;
    wifi_config.sta.password = WIFI_PASSWORD;
    wifi_config.sta.timeout_ms = 10000;
    wifi_config.auto_start  = true;
    wifi_config.auto_connect = true;

    wifi_comm = new WiFiComm("WiFi_UDP", &wifi_config, UDP_LISTEN_PORT);
    comm      = new ProtocolComm("Communication", ID_DEVICE, wifi_comm);
#endif

    comm->create_event_loop();

    comm->register_event_loop_callback(handle_set_target_vel,
                                       PROTO_CMD_SET_TARGET_VEL, nullptr);
    comm->register_event_loop_callback(handle_stop,
                                       PROTO_CMD_STOP, nullptr);
    comm->register_event_loop_callback(handle_ping,
                                       PROTO_CMD_PING, nullptr);

    // Initialize the transport (WiFi/UDP or ESP-NOW) + start protocol tasks.
    CommRet_t cret = comm->init();
    DEBUG_SERIAL("COMM", "ProtocolComm init -> %s", comm_ret_to_str(cret));

#ifdef USE_ESPNOW_TRANSPORT
    DEBUG_SERIAL("ESPNOW", "ch=%d id=0x%02X (broadcast)",
        ESPNOW_CHANNEL, (int)ID_DEVICE);
    char status_str[16];
    snprintf(status_str, sizeof(status_str), "ESP-NOW ch%d", ESPNOW_CHANNEL);
    debug_display_set_ip(status_str);
#else
    if (wifi_comm->connected) {
        wifi_ip4_t addr;
        wifi_comm->get_local_address(&addr);
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
            addr.ip[0], addr.ip[1], addr.ip[2], addr.ip[3]);
        DEBUG_SERIAL("WIFI", "Connected. IP: %s, UDP port: %d",
            ip_str, UDP_LISTEN_PORT);
        debug_display_set_ip(ip_str);
    } else {
        DEBUG_SERIAL("WIFI", "Not connected (continuing)");
    }
#endif

    // Ramp update task (100 Hz)
    xTaskCreate(motor_update_Task, "MotorUpdate", 4096, NULL, 1, NULL);

#ifndef USE_ESPNOW_TRANSPORT
    // WiFi watcher (1 Hz): logs state changes, refreshes IP, scans for SSID.
    xTaskCreate(wifi_monitor_Task, "WiFiMonitor", 4096, NULL, 1, NULL);
#endif
}

void loop() {
    vTaskDelete(NULL);
}

#endif // normal firmware path
