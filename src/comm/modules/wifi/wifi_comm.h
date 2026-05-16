/**
 * @file    wifi_comm.h
 * @brief   Comunicacao WiFi (CommBase via sockets UDP).
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 16 de Outubro de 2025
 */


#pragma once

#include "wifi_comm_utils.h"

#include "interfaces/comm_base_utils.h"
#include "interfaces/comm_base.h"

bool wifi_scan_once(
    const char* target_ssid,
    wifi_ap_record_t *out_ap
);


class WiFiComm : public CommBase {
    private:
        EventGroupHandle_t  wifi_event_group;
        wifi_comm_config_t *init_config;

        const uint8_t MAX_WIFI_RETRIES = 10;
        uint8_t wifi_retries;

        const int WIFI_CONNECTED_BIT = BIT0;
        const int WIFI_FAILED_BIT    = BIT1;

        static void wifi_event_handler(
            void* arg, esp_event_base_t event_base,
            int32_t event_id, void* event_data
        );

        static void ip_event_handler(
            void* arg, esp_event_base_t event_base,
            int32_t event_id, void* event_data
        );

        static void wifi_udp_rx_Task( void *param );
        static void wifi_udp_tx_Task( void *param );

        TaskHandle_t udp_rx_TaskHandle;
        TaskHandle_t udp_tx_TaskHandle;

        wifi_comm_udp_sock_t udp_sock;

        QueueHandle_t rx_Queue;
        QueueHandle_t tx_Queue;

        int32_t udp_port;

    public:
        char hostname[32];
        wifi_mode_t mode;
        wifi_mac_t  mac;
        wifi_ip4_t  ip;

        WiFiComm(
            const char *description = nullptr,
            wifi_comm_config_t *config = nullptr,
            uint32_t udp_port = 12345
        ) : CommBase(description),
            init_config(config),
            wifi_retries(0),
            udp_port(udp_port),
            mode(WIFI_MODE_NULL)
        {
            this->hostname[0] = '\0';
            memset( &mac, 0, sizeof(mac) );
            esp_err_t ret =
                esp_efuse_mac_get_default( this->mac.mac );
            if ( ret != ESP_OK ) {
                ESP_LOGE(
                    "WIFI INIT",
                    "Erro ao obter MAC padrao: %d",
                    ret
                );
            } else {
                ESP_LOGI(
                    "WIFI INIT",
                    "MAC %s",
                    this->mac.str
                );
            }
            mac_to_string( this->mac.mac, this->mac.str );
            memset( &ip, 0, sizeof(ip) );
        }


        CommRet_t init( ) override;
        CommRet_t deinit() override;
        CommRet_t start() override;
        CommRet_t stop() override;

        CommRet_t   send( const CommMessage_t &msg, uint32_t timeout_ms = 0 ) override;
        CommRet_t   send_to( const CommMessage_t &msg, const CommAddr_t &addr, uint32_t timeout_ms = 0 ) override;
        CommRet_t   receive( CommMessage_t &out_msg, uint32_t timeout_ms = 0 ) override;
        CommRet_t   receive_from( CommMessage_t &out_msg, CommAddr_t &addr, uint32_t timeout = 0 ) override;
        CommRet_t   peek( CommMessage_t &out_msg ) override;
        size_t      available( ) const override;


        int32_t get_rssi();

        CommRet_t set_mode( wifi_mode_t mode ){
            this->mode = mode;
            return COMM_RET_OK;
        }

        CommRet_t get_mode( wifi_mode_t *out_mode ) const {
            if ( !out_mode ) {
                return COMM_RET_INVALID_ARG;
            }
            *out_mode = mode;
            return COMM_RET_OK;
        }

        CommRet_t set_hostname( const char *hostname, size_t len ){
            if ( hostname[0] == '\0' || len == 0 || len >= sizeof(this->hostname) ) {
                return COMM_RET_INVALID_ARG;
            }
            snprintf( this->hostname, sizeof(this->hostname), "%s", hostname );
            return COMM_RET_OK;
        }

        int32_t get_hostname( char *buf, size_t buf_len ) const {
            if ( this->hostname[0] == '\0' ) {
                return COMM_RET_INVALID_ARG;
            } else if ( !buf || buf_len == 0 ) {
                return COMM_RET_INVALID_ARG;
            }
            size_t len = strlen(this->hostname);
            if ( buf_len < len + 1 ) {
                return COMM_RET_NO_MEMORY;
            }
            snprintf( buf, len + 1, "%s", this->hostname );
            return (int32_t)len;
        }

        CommRet_t get_local_address( wifi_ip4_t *out_addr ) const {
            if ( !out_addr ){
                return COMM_RET_INVALID_ARG;
            } if ( this->ip.ip[0] == 0 ) {
                return COMM_RET_IO_ERROR;
            }
            memcpy( out_addr->ip, &this->ip, sizeof( wifi_ip4_t ) );
            return COMM_RET_OK;
        }

        CommRet_t get_mac( wifi_mac_t *out_mac ) const {
            if ( !out_mac || this->mac.mac[0] == 0 ) {
                return COMM_RET_INVALID_ARG;
            }
            memcpy( out_mac->mac, this->mac.mac, sizeof( wifi_mac_t ) );
            return COMM_RET_OK;
        }

        ~WiFiComm();
};
