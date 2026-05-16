/**
 * @file    wifi_comm_utils.h
 * @brief   Tipos auxiliares e constantes para a interface Wi-Fi.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    2025
 */

#pragma once


#include "interfaces/comm_base_utils.h"
#include "interfaces/comm_base.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <string>
#include <vector>
#include <map>

#include "stdbool.h"
#include "stdint.h"
#include "stddef.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"


typedef int32_t wifi_conn_t;

static inline void mac_to_string( const uint8_t mac[6], char out[18] ) {
    snprintf(
        out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
}

static inline void ip_to_string( const uint8_t ip[4], char out[16] ) {
    snprintf(
        out, 16, "%u.%u.%u.%u",
        ip[0], ip[1], ip[2], ip[3]
    );
}

typedef enum {
    WIFI_EVT_READY = 0,
    WIFI_EVT_STARTED,
    WIFI_EVT_STOPPED,
    WIFI_EVT_STA_CONNECTED,
    WIFI_EVT_STA_DISCONNECTED,
    WIFI_EVT_GOT_IP,
    WIFI_EVT_LOST_IP,
    WIFI_EVT_SCAN_STARTED,
    WIFI_EVT_SCAN_RESULT,
    WIFI_EVT_SCAN_DONE,
    WIFI_EVT_AP_STARTED,
    WIFI_EVT_AP_STOPPED,
    WIFI_EVT_CLIENT_CONNECTED,
    WIFI_EVT_CLIENT_DISCONNECTED,
    WIFI_EVT_DATA_AVAILABLE,
    WIFI_EVT_ERROR
} wifi_event_type_t;


typedef struct {
    uint8_t ip[4];
} wifi_ip4_t;


typedef struct {
    uint8_t mac[6];
    char    str[18];
} wifi_mac_t;


typedef struct {
    wifi_ip4_t ip;
    int16_t   port;
} wifi_conn_addr_t;


typedef struct {
    int32_t sock_fd;
    struct sockaddr_in addr;
} wifi_comm_udp_sock_t;


typedef struct {
    wifi_mode_t mode;
    const char *hostname;
    struct {
        const char *ssid;
        const char *password;
        uint32_t    timeout_ms;
    } sta;
    struct {
        const char *ssid;
        const char *password;
        uint8_t     channel;
        uint8_t     max_clients;
    } ap;
    bool auto_start;
    bool auto_connect;
} wifi_comm_config_t;
