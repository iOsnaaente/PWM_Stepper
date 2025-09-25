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
    }
}

// Parse simple one-byte commands (same mapping as previous Bluetooth version)
static void handle_command(uint8_t c) {
    DEBUG_SERIAL("UDP_CMD", "Recebido: %d", c);
    switch(c) {
        case 'F': robot->drive( 1.0,  0.0 ); break; // Forward
        case 'B': robot->drive(-1.0,  0.0 ); break; // Backward
        case 'L': robot->drive( 0.0, -1.0 ); break; // Left
        case 'R': robot->drive( 0.0,  1.0 ); break; // Right
        default:  robot->drive( 0.0,  0.0 ); break; // Stop/Unknown
    }
}

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("SERIAL INIT", "Serial de  debug inicializado.");
    DEBUG_SERIAL("SERIAL INIT", "Baudrate: %d", USB_BUS_BAUDRATE);

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
                if (len > 0) incoming[len] = '\0';
                IPAddress remoteIp = Udp.remoteIP();
                if (allowed.toString() != String("0.0.0.0") && remoteIp != allowed) {
                    DEBUG_SERIAL("UDP", "Ignoring packet from %s", remoteIp.toString().c_str());
                } else {
                    for (int i = 0; i < len; ++i) {
                        handle_command((uint8_t)incoming[i]);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void loop() {
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t free_heap  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t largest    = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    DEBUG_SERIAL("RAM", "Heap total: %u KB", total_heap / 1024);
    DEBUG_SERIAL("RAM", "Heap livre: %u KB", free_heap / 1024);
    DEBUG_SERIAL("RAM", "Maior bloco livre: %u KB", largest / 1024);
    vTaskDelete(NULL);
}