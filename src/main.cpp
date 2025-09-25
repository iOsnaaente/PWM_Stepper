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
#include <lwip/inet.h>
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
Robot   *robot;

void udp_listener_Task(void *pvParameters);

WiFiUDP Udp;
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch(event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                DEBUG_SERIAL("WIFI", "Disconnected, retrying...");
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    uint32_t ip_raw = event->ip_info.ip.addr; // Little-endian
    uint8_t b1 = ip_raw & 0xFF;
    uint8_t b2 = (ip_raw >> 8) & 0xFF;
    uint8_t b3 = (ip_raw >> 16) & 0xFF;
    uint8_t b4 = (ip_raw >> 24) & 0xFF;
    DEBUG_SERIAL("WIFI", "Got IP: %u.%u.%u.%u", b1, b2, b3, b4);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        DEBUG_SERIAL("WIFI", "Connection success to SSID: %s", WIFI_SSID);
        char ipbuf[24];
        snprintf(ipbuf, sizeof(ipbuf), "%u.%u.%u.%u", b1, b2, b3, b4);
        debug_display_set_ip(ipbuf);
    }
}

// ----------------------------------------------------------------------------------
// Control packet parsing for analog commands from an Xbox controller (or similar)
// Supported formats:
// 1) Binary compact: [int8 vel][int8 turn]            (2 bytes)
//    Optional checksum byte: xor(vel, turn)           (3 bytes)
//    Mapping: -128..127 (excluding -128) -> -1.0..1.0 (value / 127.0f)
// 2) ASCII CSV: "<vel>,<turn>" e.g. "0.62,-0.15" (null terminator optional)
// 3) Legacy single-letter fallback (F,B,L,R, any other = stop)
// A watchdog stops the robot if no valid command arrives within COMMAND_TIMEOUT_MS.

static const uint32_t COMMAND_TIMEOUT_MS = 500; // auto-stop timeout
static TickType_t last_command_tick = 0;         // last time a valid command was applied

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

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("SERIAL INIT", "Serial de  debug inicializado.");
    DEBUG_SERIAL("SERIAL INIT", "Baudrate: %d", USB_BUS_BAUDRATE);

    // Inicializa display de debug (baseline + overlay)
    debug_display_init();
    DEBUG_SERIAL("OLED", "Display debug inicializado");

    motor_esquerdo = new Stepper( M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0 );
    motor_direito  = new Stepper( M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_1, LEDC_TIMER_0 );
    robot = new Robot( *motor_esquerdo, *motor_direito );
    robot->stop();

    // ---- WiFi Initialization ----
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    DEBUG_SERIAL("WIFI", "Connecting to SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        DEBUG_SERIAL("WIFI", "Failed to connect within timeout, continuing anyway");
    }

    if (Udp.begin(UDP_LISTEN_PORT)) {
        DEBUG_SERIAL("UDP", "Listening on port %d", UDP_LISTEN_PORT);
    } else {
        DEBUG_SERIAL("UDP", "Failed to start UDP on port %d", UDP_LISTEN_PORT);
    }

    xTaskCreatePinnedToCore(udp_listener_Task, "UDPControl", 8192, NULL, 1, NULL, 0);
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
                            robot->drive_wheels(left, right);
                            last_command_tick = xTaskGetTickCount();
                            DEBUG_SERIAL("CTRL", "L=%.2f R=%.2f", left, right);
                        } else {
                            DEBUG_SERIAL("UDP", "Invalid packet (len=%d)", len);
                        }
                    }
                }
            }
        }
        // Watchdog timeout -> stop motors
        if (last_command_tick != 0) {
            TickType_t now = xTaskGetTickCount();
            if ( (now - last_command_tick) * portTICK_PERIOD_MS > COMMAND_TIMEOUT_MS ) {
                robot->drive_wheels(0.0f, 0.0f);
                last_command_tick = 0; // prevent re-entering until new cmd
                DEBUG_SERIAL("CTRL", "Timeout stop");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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