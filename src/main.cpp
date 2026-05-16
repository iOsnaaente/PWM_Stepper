/**
  * @file PWM_Controller.cpp
  * @author Bruno Gabriel Flores Sampaio
  * @author Icaro Marques de Campos 
  * @date 13 de Junho de 2025
 **/

// ----------------------------------------------------------------------------------
// main.cpp - WiFi/UDP control (formerly Bluetooth)
// ----------------------------------------------------------------------------------

#include "../board_config.h"

#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "Display/debug_display.h"

#ifndef WIFI_SSID
#error "WIFI_SSID não definido. Copie include/credentials.template.h para include/credentials.h e preencha as credenciais."
#endif
#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD não definido. Copie include/credentials.template.h para include/credentials.h e preencha as credenciais."
#endif
#ifndef UDP_LISTEN_PORT
#error "UDP_LISTEN_PORT não definido em credentials.h"
#endif
#ifndef UDP_ALLOWED_REMOTE_IP
#error "UDP_ALLOWED_REMOTE_IP não definido em credentials.h"
#endif

Stepper *motor_esquerdo;
Stepper *motor_direito;
void motor_update_Task(void *pvParameters);
Robot   *robot;

void udp_listener_Task(void *pvParameters);

WiFiUDP Udp;

// ----------------------------------------------------------------------------------
// Control packet parsing for analog commands from an Xbox controller (or similar)
// Supported formats:
// 1) Binary compact: [int8 vel][int8 turn]            (2 bytes)
//    Optional checksum byte: xor(vel, turn)           (3 bytes)
//    Mapping: -128..127 (excluding -128) -> -1.0..1.0 (value / 127.0f)
// 2) ASCII CSV: "<vel>,<turn>" e.g. "0.62,-0.15" (null terminator optional)
// 3) Legacy single-letter fallback (F,B,L,R, any other = stop)
// A watchdog stops the robot if no valid command arrives within COMMAND_TIMEOUT_MS.

static const uint32_t COMMAND_TIMEOUT_MS   = 500;  // auto-stop timeout
static TickType_t     last_command_tick    = 0;    // last time a valid command was applied
static const float    CMD_DEADZONE         = 0.05f; // treat small magnitudes as zero
static const uint32_t WIFI_START_DELAY_MS  = 3000; // delay before bringing up WiFi (ms)

static inline float clamp_unit(float v) { return fminf(fmaxf(v, -1.0f), 1.0f); }

// Binary / ASCII now represent LEFT,RIGHT wheel velocities directly
static bool parse_binary_packet(const uint8_t* data, int len, float& left, float& right) {
    if (len != 2 && len != 3) return false;
    int8_t v8 = (int8_t)data[0];
    int8_t t8 = (int8_t)data[1];
    if (len == 3) {
        uint8_t checksum = data[2];
        if (((uint8_t)(v8 ^ t8)) != checksum) return false; // checksum mismatch
    }
    // Avoid -128 edge (keep symmetric range)
    if (v8 == -128) v8 = -127;
    if (t8 == -128) t8 = -127;
    left  = clamp_unit((float)v8 / 127.0f);
    right = clamp_unit((float)t8 / 127.0f);
    return true;
}

static bool parse_ascii_packet(char* buf, int len, float& left, float& right) {
    // Ensure null termination
    buf[len] = '\0';
    char* comma = strchr(buf, ',');
    if (!comma) return false;
    *comma = '\0';
    char* first  = buf;
    char* second = comma + 1;
    char* endptr1 = nullptr; char* endptr2 = nullptr;
    float l = strtof(first, &endptr1);
    float r = strtof(second, &endptr2);
    if (endptr1 == first || endptr2 == second) return false; // parse failure
    left  = clamp_unit(l);
    right = clamp_unit(r);
    return true;
}

static bool parse_legacy_single(const uint8_t* data, int len, float& left, float& right) {
    if (len != 1) return false;
    uint8_t c = data[0];
    switch(c) {
        case 'F': left =  1.0f; right =  1.0f; return true; // forward
        case 'B': left = -1.0f; right = -1.0f; return true; // backward
        case 'L': left = -0.5f; right =  0.5f; return true; // pivot left
        case 'R': left =  0.5f; right = -0.5f; return true; // pivot right
        default:  left = 0.0f;  right = 0.0f;  return true; // stop
    }
}

static bool decode_control_packet(uint8_t* data, int len, float& left, float& right) {
    // Try binary compact first
    if (parse_binary_packet(data, len, left, right)) return true;
    // Try ASCII (only if it contains comma or dot for floats)
    if (memchr(data, ',', len)) {
        return parse_ascii_packet((char*)data, len, left, right);
    }
    // Fallback legacy
    return parse_legacy_single(data, len, left, right);
}

// Shared watchdog: stop motors and disable torque if no command within timeout
static void control_watchdog_step() {
    if (last_command_tick != 0) {
        TickType_t now = xTaskGetTickCount();
        if ( (now - last_command_tick) * portTICK_PERIOD_MS > COMMAND_TIMEOUT_MS ) {
            robot->drive_wheels(0.0f, 0.0f);
            // To reduce heat, disable torque when idle
            robot->set_torque(false);
            last_command_tick = 0; // prevent re-entering until new cmd
            DEBUG_SERIAL("CTRL", "Timeout stop");
        }
    }
}

#if !defined(MOTOR_SELF_TEST) && !defined(USE_ESPNOW_CONTROL) && !defined(GY521_SELF_TEST) && !defined(KINEMATICS_SELF_TEST) && !defined(STRAIGHT_SELF_TEST)
void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("RESET", "Reset reason: %d", (int)esp_reset_reason());
    DEBUG_SERIAL("SERIAL INIT", "Serial de  debug inicializado.");
    DEBUG_SERIAL("SERIAL INIT", "Baudrate: %d", USB_BUS_BAUDRATE);

    // Inicializa display de debug (baseline + overlay)
    debug_display_init();
    DEBUG_SERIAL("OLED", "Display debug inicializado");

        // Left motor: DRV8825 (microstep setting defined by wiring; keeping 1 here)
        // Use separate LEDC timers so each wheel can run its own step frequency
        motor_esquerdo = new Stepper( M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0, Stepper::DRIVER_DRV8825, 1, false );
        // Right motor: DRV8825 (inverted to match physical mounting)
        motor_direito  = new Stepper( M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_1, LEDC_TIMER_1, Stepper::DRIVER_DRV8825, 1, true );
    robot = new Robot( *motor_esquerdo, *motor_direito );
    robot->stop();

    // Pequeno atraso antes de iniciar o WiFi para reduzir pico de corrente na partida
    DEBUG_SERIAL("POWER", "Aguardando %lu ms antes de iniciar WiFi", (unsigned long)WIFI_START_DELAY_MS);
    delay(WIFI_START_DELAY_MS);

    // ---- WiFi Initialization (simplified Arduino flow) ----
    DEBUG_SERIAL("WIFI", "Connecting to SSID: %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    // Reduce WiFi TX power to lower current spikes on weak supplies
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 10000UL) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        DEBUG_SERIAL("WIFI", "Connected. IP: %s", ip.toString().c_str());
        debug_display_set_ip(ip.toString().c_str());
    } else {
        DEBUG_SERIAL("WIFI", "Failed to connect within timeout, continuing anyway");
    }

    if (Udp.begin(UDP_LISTEN_PORT)) {
        DEBUG_SERIAL("UDP", "Listening on port %d", UDP_LISTEN_PORT);
    } else {
        DEBUG_SERIAL("UDP", "Failed to start UDP on port %d", UDP_LISTEN_PORT);
    }

    // On ESP32-C3 there is only one core; use normal task creation
    xTaskCreate(udp_listener_Task, "UDPControl", 8192, NULL, 1, NULL);
    // Ramping update task (100 Hz)
    xTaskCreate(motor_update_Task, "MotorUpdate", 4096, NULL, 1, NULL);
}

void udp_listener_Task(void *pvParameters) {
    char incoming[UDP_MAX_PACKET_SIZE + 1];
    IPAddress allowed; allowed.fromString(UDP_ALLOWED_REMOTE_IP);
    for(;;) {
        int packetSize = Udp.parsePacket();
        if (packetSize > 0) {
            if (packetSize > UDP_MAX_PACKET_SIZE) {
                while (Udp.available()) Udp.read();
                DEBUG_SERIAL("UDP", "Packet too large: %d", packetSize);
            } else {
                int len = Udp.read((uint8_t*)incoming, UDP_MAX_PACKET_SIZE);
                if (len > 0) {
                    IPAddress remoteIp = Udp.remoteIP();
                    if (allowed.toString() != String("0.0.0.0") && remoteIp != allowed) {
                        DEBUG_SERIAL("UDP", "Ignoring packet from %s", remoteIp.toString().c_str());
                    } else {
                        float left=0.0f, right=0.0f;
                        if (decode_control_packet((uint8_t*)incoming, len, left, right)) {
                            // Deadzone to treat near-zero as zero
                            float lcmd = (fabsf(left)  < CMD_DEADZONE) ? 0.0f : left;
                            float rcmd = (fabsf(right) < CMD_DEADZONE) ? 0.0f : right;

                            if (lcmd == 0.0f && rcmd == 0.0f) {
                                // Both near zero -> fully stop and disable torque
                                robot->drive_wheels(0.0f, 0.0f);
                                robot->set_torque(false);
                                DEBUG_SERIAL("CTRL", "Near-zero both -> motors disabled");
                            } else {
                                // Shared enable pin: keep torque enabled when any wheel should move
                                if (!robot->get_torque()) robot->set_torque(true);
                                robot->drive_wheels(lcmd, rcmd);
                                DEBUG_SERIAL("CTRL", "L=%.2f R=%.2f (dz)", lcmd, rcmd);
                            }
                            last_command_tick = xTaskGetTickCount();
                        } else {
                            DEBUG_SERIAL("UDP", "Invalid packet (len=%d)", len);
                        }
                    }
                }
            }
        }
        control_watchdog_step();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void motor_update_Task(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    for(;;) {
        motor_esquerdo->update(0.010f);
        motor_direito->update(0.010f);
        control_watchdog_step();
        vTaskDelayUntil(&last, period);
    }
}


void loop() {
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t free_heap  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    // DEBUG_SERIAL("RAM", "Heap total: %u KB", total_heap / 1024);
    // DEBUG_SERIAL("RAM", "Heap livre: %u KB", free_heap / 1024);
    // DEBUG_SERIAL("RAM", "Maior bloco livre: %u KB", largest / 1024); // (Comentado conforme solicitado)
    vTaskDelete(NULL);
}
#endif // !MOTOR_SELF_TEST && !USE_ESPNOW_CONTROL