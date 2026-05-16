/**
 * @file    wifi_comm.cpp
 * @brief   Comunicacao WiFi (CommBase via sockets UDP).
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 16 de Outubro de 2025
 */

#include "wifi_comm.h"

void WiFiComm::wifi_udp_rx_Task( void *pvParameters ){
    WiFiComm* comm = static_cast<WiFiComm*>( pvParameters );
    if ( !comm ){
        ESP_LOGE("WIFI UDP RX", "WiFiComm instance is null in RX task");
        vTaskDelete(NULL);
        return;
    }
    if ( !comm->rx_Queue ) {
        ESP_LOGE("WIFI UDP RX", "RX Queue is null in RX task");
        vTaskDelete(NULL);
        return;
    }
    int32_t sock = comm->udp_sock.sock_fd;
    if ( sock < 0 ) {
        ESP_LOGE("WIFI UDP RX", "Socket UDP invalido na task de recepcao");
        vTaskDelete(NULL);
        return;
    }
    uint8_t rx_buffer[COMM_MAX_PAYLOAD];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    CommMessage_t msg_buff;

    while ( true ) {
        if (
            comm->udp_sock.sock_fd < 0 ||
            comm->get_state() != COMM_STATE_RUNNING
        ) {
            ESP_LOGI("WIFI UDP RX", "Encerrando task de recepcao UDP");
            vTaskDelay( pdMS_TO_TICKS(250) );
            continue;
        }
        socklen = sizeof(source_addr);
        ssize_t recv_len = recvfrom(
            sock, rx_buffer, sizeof(rx_buffer), 0,
            (struct sockaddr *)&source_addr, &socklen
        );
        if ( recv_len > 0 ){
            memset(&msg_buff, 0, sizeof(msg_buff));
            msg_buff.length =
                (uint32_t)recv_len < COMM_MAX_PAYLOAD
                    ? (size_t)recv_len
                    : COMM_MAX_PAYLOAD;
            memcpy( msg_buff.payload, rx_buffer, msg_buff.length );
            if ( msg_buff.length > 0 ) {
                msg_buff.type = (CommMessageType_t)rx_buffer[0];
            } else {
                msg_buff.type = COMM_MSG_TYPE_UNKNOWN;
            }
            CommAddr_t src_addr;
            src_addr.type = COMM_TYPE_WIFI;
            memcpy( src_addr.data, &source_addr.sin_addr.s_addr, 4 );
            uint16_t port_host = ntohs(source_addr.sin_port);
            memcpy( src_addr.data + 4, &port_host, 2 );
            src_addr.len = 6;
            msg_buff.src = src_addr;
            msg_buff.dest = CommAddr_t{};
            if (
                xQueueSend(
                    comm->rx_Queue,
                    &msg_buff,
                    pdMS_TO_TICKS(100)
                ) != pdTRUE
            ) {
                ESP_LOGW("WIFI UDP RX", "Fila RX cheia descartando pacote de %d bytes",
                    (int)recv_len );
                CommMessage_t discarded_msg;
                if ( xQueueReceive( comm->rx_Queue, &discarded_msg, 0 ) == pdTRUE ) {
                    xQueueSend( comm->rx_Queue, &msg_buff, 0 );
                }
            }
            comm->incr_msg_counter();
            comm->msg_received++;
        }
        else if ( recv_len == -1 ) {
            if ( errno == EINTR ) {
                continue;
            }
            else {
                ESP_LOGE("WIFI UDP RX", "Erro no socket UDP - Ret: %s", strerror(errno));
                vTaskDelay( pdMS_TO_TICKS(250) );
            }
        }else {
            vTaskDelay( pdMS_TO_TICKS(250) );
        }
    }
    vTaskDelete(NULL);
}


void WiFiComm::wifi_udp_tx_Task( void *pvParameters ){
    WiFiComm* comm = static_cast<WiFiComm*>( pvParameters );
    if ( !comm ){
        ESP_LOGE("WIFI UDP TX", "WiFiComm instance is null in TX task");
        vTaskDelete(NULL);
        return;
    }
    if ( !comm->tx_Queue ) {
        ESP_LOGE("WIFI UDP TX", "tx_Queue is null in TX task");
        vTaskDelete(NULL);
        return;
    }
    if ( comm->udp_sock.sock_fd < 0 ) {
        ESP_LOGE("WIFI UDP TX", "Socket UDP invalido na TX task");
        vTaskDelete(NULL);
        return;
    }
    CommMessage_t msg_buff;
    struct sockaddr_in dest_addr;
    while ( true ) {
        if (
            comm->udp_sock.sock_fd < 0 ||
            comm->get_state() != COMM_STATE_RUNNING
        ) {
            vTaskDelay( pdMS_TO_TICKS(250) );
            continue;
        }
        if ( xQueueReceive( comm->tx_Queue, &msg_buff, portMAX_DELAY ) == pdTRUE ) {
            if ( comm->udp_sock.sock_fd < 0 ) {
                continue;
            }
            memset( &dest_addr, 0, sizeof(dest_addr) );
            dest_addr.sin_family = AF_INET;
            uint16_t dest_port_host = comm->udp_port;

            if (
                msg_buff.dest.type == COMM_TYPE_WIFI &&
                msg_buff.dest.len >= 6
            ) {
                char ip_str[16];
                snprintf(
                    ip_str, sizeof(ip_str),
                    "%u.%u.%u.%u",
                    (unsigned)msg_buff.dest.data[0],
                    (unsigned)msg_buff.dest.data[1],
                    (unsigned)msg_buff.dest.data[2],
                    (unsigned)msg_buff.dest.data[3]
                );
                dest_addr.sin_addr.s_addr = inet_addr( ip_str );
                uint16_t port_host;
                memcpy( &port_host, msg_buff.dest.data + 4, 2 );
                dest_addr.sin_port = htons( port_host );
                dest_port_host = port_host;
            }
            else if (
                msg_buff.dest.type == COMM_TYPE_WIFI &&
                msg_buff.dest.len >= 4
            ) {
                char ip_str[16];
                snprintf(
                    ip_str, sizeof(ip_str),
                    "%u.%u.%u.%u",
                    (unsigned)msg_buff.dest.data[0],
                    (unsigned)msg_buff.dest.data[1],
                    (unsigned)msg_buff.dest.data[2],
                    (unsigned)msg_buff.dest.data[3]
                );
                dest_addr.sin_addr.s_addr = inet_addr( ip_str );
                dest_addr.sin_port = htons( comm->udp_port );
                dest_port_host = comm->udp_port;
            }
            else {
                dest_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
                dest_addr.sin_port = htons( comm->udp_port );
                dest_port_host = comm->udp_port;
            }
            ssize_t sent_len = sendto(
                comm->udp_sock.sock_fd,
                msg_buff.payload,
                msg_buff.length,
                0,
                (struct sockaddr *)&dest_addr,
                sizeof(dest_addr)
            );
            if ( sent_len < 0 ) {
                ESP_LOGE("WIFI UDP TX", "Erro ao enviar mensagem UDP - Ret: %s",
                    strerror(errno) );
            }
            else if ( (size_t)sent_len < msg_buff.length ) {
                ESP_LOGW("WIFI UDP TX", "Mensagem UDP parcialmente enviada (%d de %d bytes)",
                    (int)sent_len, (int)msg_buff.length );
            }
            else {
                (void)dest_port_host;
                comm->incr_msg_counter();
                comm->msg_sent++;
            }
        }
    }
    vTaskDelete(NULL);
}


void WiFiComm::wifi_event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data
) {
    WiFiComm* comm = static_cast<WiFiComm*>(arg);
    if (!comm) return;
    switch(event_id) {
        case WIFI_EVENT_STA_START:{
            ESP_LOGI("WIFI EVENT", "STA inicializando. Tentando conectar...");
            esp_wifi_connect();
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:{
            wifi_event_sta_disconnected_t* dis =
                (wifi_event_sta_disconnected_t*) event_data;
            comm->connected = false;
            xEventGroupClearBits(
                comm->wifi_event_group,
                comm->WIFI_CONNECTED_BIT
            );
            if (
                comm->init_config->auto_connect &&
                (comm->wifi_retries++) < comm->MAX_WIFI_RETRIES
            ) {
                ESP_LOGW("WIFI EVENT",
                    "REASON[%d]: STA desconectada. Tentando reconectar...",
                    dis->reason);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(
                    comm->wifi_event_group,
                    comm->WIFI_FAILED_BIT
                );
                ESP_LOGE("WIFI EVENT",
                    "Falha ao conectar no AP apos %d tentativas.",
                    comm->MAX_WIFI_RETRIES);
            }
            break;
        }
        default:
            break;
    }
    return;
}


void WiFiComm::ip_event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data
){
    WiFiComm* comm = static_cast<WiFiComm*>(arg);
    if ( !comm ) return;
    switch(event_id) {
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t* data =
                (ip_event_got_ip_t*) event_data;
            memcpy(
                &comm->ip.ip,
                &data->ip_info.ip,
                sizeof(wifi_ip4_t)
            );
            ESP_LOGI("IP EVENT", "Station got IP - IP:" IPSTR,
                IP2STR(&data->ip_info.ip));
            // Mark connection good for any caller polling these fields and
            // reset the retry counter so future disconnects get a fresh budget.
            comm->connected    = true;
            comm->wifi_retries = 0;
            xEventGroupSetBits(
                comm->wifi_event_group,
                comm->WIFI_CONNECTED_BIT
            );
            break;
        }
        case IP_EVENT_AP_STAIPASSIGNED: {
            ip_event_ap_staipassigned_t* data =
                (ip_event_ap_staipassigned_t*) event_data;
            ESP_LOGI("IP EVENT",
                "AP assigned IP " IPSTR,
                IP2STR(&data->ip));
            break;
        }
        default:
            break;
    }
}




CommRet_t WiFiComm::init( ) {
    if ( !init_config ) {
        ESP_LOGE("WIFI INIT", "Configuracao de inicializacao nao fornecida");
        return COMM_RET_INVALID_ARG;
    }
    this->mode = init_config->mode;
    if ( this->mode == WIFI_MODE_NULL ) {
        ESP_LOGW("WIFI INIT", "Modo WiFi NULL");
        return COMM_RET_INVALID_ARG;
    }
    this->wifi_event_group = xEventGroupCreate();
    if ( this->wifi_event_group == NULL ) {
        ESP_LOGE("WIFI START", "Erro ao criar grupo de eventos WiFi");
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    this->rx_Queue = xQueueCreate( 10, sizeof(CommMessage_t) );
    this->tx_Queue = xQueueCreate( 10, sizeof(CommMessage_t) );
    if ( !this->rx_Queue || !this->tx_Queue ) {
        ESP_LOGE("WIFI START", "Erro ao criar filas de dados WiFi");
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }

    esp_err_t ret;
    ret = nvs_flash_init();
    if (   ret == ESP_ERR_NVS_NO_FREE_PAGES
        || ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        ret = nvs_flash_erase();
        if ( ret != ESP_OK ) {
            ESP_LOGE("WIFI INIT", "Erro ao apagar NVS - Ret: %s",
                esp_err_to_name(ret) );
            return COMM_RET_IO_ERROR;
        }
        ret = nvs_flash_init();
        if ( ret != ESP_OK ) {
            ESP_LOGE("WIFI INIT", "Erro ao inicializar NVS - Ret: %s",
                esp_err_to_name(ret) );
            return COMM_RET_IO_ERROR;
        }
    } else if ( ret != ESP_OK ) {
        // Tolera ja inicializado (Arduino pode te-lo feito).
        ESP_LOGW("WIFI INIT", "nvs_flash_init: %s (continuando)",
            esp_err_to_name(ret) );
    }

    ret = esp_netif_init();
    if ( ret != ESP_OK && ret != ESP_ERR_INVALID_STATE ) {
        ESP_LOGE("WIFI INIT", "Erro ao inicializar esp_netif - Ret: %s",
            esp_err_to_name(ret) );
        return COMM_RET_IO_ERROR;
    }

    ret = esp_event_loop_create_default();
    if ( ret != ESP_OK && ret != ESP_ERR_INVALID_STATE ) {
        ESP_LOGE("WIFI INIT", "Erro ao criar event loop padrao - Ret: %s",
            esp_err_to_name(ret) );
        return COMM_RET_IO_ERROR;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init( &cfg );
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI INIT", "Erro ao inicializar driver WiFi - Ret: %s",
            esp_err_to_name(ret) );
        return COMM_RET_IO_ERROR;
    }
    ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        (void*)this,
        NULL
    );
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI START", "Erro ao registrar handler WiFi - Ret: %s",
            esp_err_to_name(ret) );
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    ret = esp_event_handler_instance_register(
        IP_EVENT, ESP_EVENT_ANY_ID,
        &ip_event_handler,
        (void*)this,
        NULL
    );
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI START", "Erro ao registrar handler IP - Ret: %s",
            esp_err_to_name(ret) );
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    ret = esp_wifi_set_mode( this->mode );
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI INIT", "Erro ao configurar modo WiFi - Ret: %s",
            esp_err_to_name(ret) );
        return COMM_RET_IO_ERROR;
    }
    esp_netif_t *net_interface = nullptr;
    if (
        this->mode == WIFI_MODE_STA ||
        this->mode == WIFI_MODE_APSTA
    ){
        net_interface = esp_netif_create_default_wifi_sta();
        if (
            init_config->hostname != nullptr &&
            strlen(init_config->hostname) > 0
        ) {
            snprintf(
                this->hostname,
                sizeof(this->hostname),
                "%s",
                init_config->hostname
            );
            ret = esp_netif_set_hostname( net_interface, this->hostname );
            if ( ret != ESP_OK ) {
                ESP_LOGW("WIFI INIT", "Erro ao configurar hostname - Ret: %s",
                    esp_err_to_name(ret) );
            }
        }
        wifi_config_t wifi_config = {};
        if (init_config->sta.ssid){
            strncpy(
                (char*)wifi_config.sta.ssid,
                init_config->sta.ssid,
                sizeof(wifi_config.sta.ssid)
            );
            ESP_LOGI("WIFI INIT", "STA - SSID: %s", wifi_config.sta.ssid);
        } else {
            ESP_LOGE("WIFI INIT", "STA - SSID nao fornecido");
            return COMM_RET_INVALID_ARG;
        }
        if (init_config->sta.password){
            strncpy(
                (char*)wifi_config.sta.password,
                init_config->sta.password,
                sizeof(wifi_config.sta.password)
            );
        }
        ret = esp_wifi_set_config( WIFI_IF_STA, &wifi_config );
        if ( ret != ESP_OK ) {
            ESP_LOGE("WIFI INIT", "STA - Erro ao configurar credenciais - Ret: %s",
                esp_err_to_name(ret) );
            return COMM_RET_IO_ERROR;
        }
    }
    if (
        this->mode == WIFI_MODE_AP ||
        this->mode == WIFI_MODE_APSTA
    ){
        net_interface = esp_netif_create_default_wifi_ap();
        if (
            init_config->hostname != nullptr &&
            strlen(init_config->hostname) > 0
        ) {
            snprintf(
                this->hostname,
                sizeof(this->hostname),
                "%s",
                init_config->hostname
            );
            ret = esp_netif_set_hostname( net_interface, this->hostname );
            if ( ret != ESP_OK ) {
                ESP_LOGW("WIFI INIT", "Erro ao configurar hostname - Ret: %s",
                    esp_err_to_name(ret) );
            }
        }
        wifi_config_t wifi_config = {};
        if (init_config->ap.ssid) {
            strncpy(
                (char*)wifi_config.ap.ssid,
                init_config->ap.ssid,
                sizeof(wifi_config.ap.ssid)
            );
            wifi_config.ap.ssid_len = strlen( init_config->ap.ssid );
        } else {
            ESP_LOGE("WIFI INIT", "AP - SSID nao fornecido");
            return COMM_RET_INVALID_ARG;
        }
        if (init_config->ap.password){
            strncpy(
                (char*)wifi_config.ap.password,
                init_config->ap.password,
                sizeof(wifi_config.ap.password)
            );
            wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        } else {
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }
        wifi_config.ap.channel = init_config->ap.channel;
        wifi_config.ap.max_connection = init_config->ap.max_clients;
        ret = esp_wifi_set_config( WIFI_IF_AP, &wifi_config );
        if ( ret != ESP_OK ) {
            ESP_LOGE("WIFI INIT", "Erro ao configurar AP - Ret: %s",
                esp_err_to_name(ret) );
            return COMM_RET_IO_ERROR;
        }
    }
    ESP_LOGI("WIFI INIT", "Modulo WiFi inicializado com sucesso");
    if ( init_config->auto_start ) {
        return this->start();
    }
    this->set_state( COMM_STATE_IDLE );
    this->set_last_error( COMM_RET_OK );
    return COMM_RET_OK;
}


CommRet_t WiFiComm::start() {
    if ( this->get_state() == COMM_STATE_RUNNING ) {
        ESP_LOGW("WIFI START", "Modulo WiFi ja esta em execucao");
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE("WIFI START", "Erro ao iniciar driver WiFi - Ret: %s",
            esp_err_to_name(ret));
        return COMM_RET_IO_ERROR;
    }
    if ( this->init_config->auto_connect ){
        EventBits_t bits = xEventGroupWaitBits(
            this->wifi_event_group,
            this->WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
            pdMS_TO_TICKS(10000)
        );
        if (bits & this->WIFI_CONNECTED_BIT) {
            ESP_LOGI("WIFI START", "Conexao WiFi estabelecida com sucesso!");
            this->connected = true;
        } else {
            ESP_LOGE("WIFI START", "Falha ao conectar no WiFi");
            this->set_last_error( COMM_RET_IO_ERROR );
            this->set_state( COMM_STATE_FAULT );
            this->connected = false;
        }
    }
    this->udp_sock.sock_fd =
        socket( AF_INET, SOCK_DGRAM, IPPROTO_IP );
    if ( this->udp_sock.sock_fd < 0 ) {
        ESP_LOGE("WIFI START", "Erro ao criar socket UDP");
        this->set_last_error( COMM_RET_IO_ERROR );
        this->set_state( COMM_STATE_FAULT );
        this->connected = false;
        return COMM_RET_IO_ERROR;
    }
    int broadcast_enable = 1;
    setsockopt(
        this->udp_sock.sock_fd,
        SOL_SOCKET,
        SO_BROADCAST,
        &broadcast_enable,
        sizeof(broadcast_enable)
    );
    this->udp_sock.addr.sin_family = AF_INET;
    this->udp_sock.addr.sin_port = htons( this->udp_port );
    this->udp_sock.addr.sin_addr.s_addr = htonl( INADDR_ANY );
    int bind_ret = bind(
        this->udp_sock.sock_fd,
        (struct sockaddr*)&this->udp_sock.addr,
        sizeof(this->udp_sock.addr)
    );
    if ( bind_ret < 0 ) {
        ESP_LOGE("WIFI START", "Erro ao fazer bind do socket UDP");
        this->set_last_error( COMM_RET_IO_ERROR );
        this->connected = false;
        return COMM_RET_IO_ERROR;
    }
    ESP_LOGI("WIFI START", "Socket UDP criado e bindado");

    // Set state RUNNING *before* spawning tasks so they don't bail
    if (this->connected) {
        this->set_state( COMM_STATE_RUNNING );
    }

    xTaskCreate(
        wifi_udp_rx_Task,
        "wifi_udp_rx_Task",
        4096,
        (void*)this,
        tskIDLE_PRIORITY + 5,
        &this->udp_rx_TaskHandle
    );
    if ( this->udp_rx_TaskHandle == NULL ) {
        ESP_LOGE("WIFI START", "Erro ao criar task de RX UDP");
        this->set_last_error( COMM_RET_IO_ERROR );
        this->connected = false;
        return COMM_RET_IO_ERROR;
    }
    xTaskCreate(
        wifi_udp_tx_Task,
        "wifi_udp_tx_Task",
        4096,
        (void*)this,
        tskIDLE_PRIORITY + 5,
        &this->udp_tx_TaskHandle
    );
    if ( this->udp_tx_TaskHandle == NULL ) {
        ESP_LOGE("WIFI START", "Erro ao criar task de TX UDP");
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    this->set_last_error( COMM_RET_OK );
    this->connected = true;
    return COMM_RET_OK;
}

CommRet_t WiFiComm::deinit() {
    CommRet_t stop_ret = this->stop();
    if ( stop_ret != COMM_RET_OK ) {
        ESP_LOGE("WIFI DEINIT", "Erro ao parar o modulo WiFi");
        return stop_ret;
    }
    esp_err_t ret = esp_wifi_deinit();
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI DEINIT", "Erro ao desinicializar driver WiFi - Ret: %s",
            esp_err_to_name(ret) );
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    this->set_state( COMM_STATE_STOPPED );
    this->set_last_error( COMM_RET_OK );
    return COMM_RET_OK;
}

CommRet_t WiFiComm::stop() {
    if ( this->get_state() != COMM_STATE_RUNNING ) {
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    esp_err_t ret = esp_wifi_stop();
    if ( ret != ESP_OK ) {
        ESP_LOGE("WIFI STOP", "Erro ao parar driver WiFi - Ret: %s",
            esp_err_to_name(ret) );
        this->set_last_error( COMM_RET_IO_ERROR );
        return COMM_RET_IO_ERROR;
    }
    if ( this->udp_rx_TaskHandle ) {
        vTaskDelete( this->udp_rx_TaskHandle );
        this->udp_rx_TaskHandle = NULL;
    }
    if ( this->udp_tx_TaskHandle ) {
        vTaskDelete( this->udp_tx_TaskHandle );
        this->udp_tx_TaskHandle = NULL;
    }
    if ( this->udp_sock.sock_fd >= 0 ) {
        shutdown( this->udp_sock.sock_fd, SHUT_RDWR );
        close( this->udp_sock.sock_fd );
        this->udp_sock.sock_fd = -1;
    }
    if ( this->rx_Queue ) {
        vQueueDelete( this->rx_Queue );
        this->rx_Queue = NULL;
    }
    if ( this->tx_Queue ) {
        vQueueDelete( this->tx_Queue );
        this->tx_Queue = NULL;
    }
    this->set_state( COMM_STATE_STOPPED );
    this->set_last_error( COMM_RET_OK );
    return COMM_RET_OK;
}

CommRet_t WiFiComm::send(
    const CommMessage_t &msg,
    uint32_t timeout_ms
) {
    if ( !this->tx_Queue ) {
        this->set_last_error( COMM_RET_NOT_INITIALIZED );
        return COMM_RET_NOT_INITIALIZED;
    }
    TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
    CommMessage_t msg_to_send = msg;
    msg_to_send.dest.type = COMM_TYPE_WIFI;
    msg_to_send.dest.data[0] = 255;
    msg_to_send.dest.data[1] = 255;
    msg_to_send.dest.data[2] = 255;
    msg_to_send.dest.data[3] = 255;
    msg_to_send.dest.len = 4;
    if ( xQueueSend(
        this->tx_Queue,
        &msg_to_send,
        ticks_to_wait
    ) == pdTRUE ) {
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    this->set_last_error( COMM_RET_TIMEOUT );
    return COMM_RET_TIMEOUT;
}

CommRet_t WiFiComm::send_to(
    const CommMessage_t &msg,
    const CommAddr_t &addr,
    uint32_t timeout_ms
) {
    if ( !this->tx_Queue ) {
        this->set_last_error( COMM_RET_NOT_INITIALIZED );
        return COMM_RET_NOT_INITIALIZED;
    }
    CommMessage_t msg_to_send = msg;
    msg_to_send.dest = addr;
    TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout_ms);
    if ( xQueueSend(
        this->tx_Queue,
        &msg_to_send,
        ticks_to_wait
    ) == pdTRUE ) {
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    this->set_last_error( COMM_RET_TIMEOUT );
    return COMM_RET_TIMEOUT;
}

CommRet_t WiFiComm::receive(
    CommMessage_t &out_msg,
    uint32_t timeout_ms
) {
    if ( !this->rx_Queue ) {
        this->set_last_error( COMM_RET_NOT_INITIALIZED );
        return COMM_RET_NOT_INITIALIZED;
    }
    TickType_t ticks_to_wait =
        ( timeout_ms == 0 )
            ? portMAX_DELAY
            : pdMS_TO_TICKS(timeout_ms);
    if (
        xQueueReceive(
            this->rx_Queue,
            &out_msg,
            ticks_to_wait
        ) == pdTRUE
    ) {
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    this->set_last_error( COMM_RET_TIMEOUT );
    return COMM_RET_TIMEOUT;
}

CommRet_t WiFiComm::receive_from(
    CommMessage_t &out_msg,
    CommAddr_t &addr,
    uint32_t timeout
) {
    (void)out_msg; (void)addr; (void)timeout;
    return COMM_RET_NOT_IMPLEMENTED;
}

CommRet_t WiFiComm::peek( CommMessage_t &out_msg ) {
    if ( !this->rx_Queue ) {
        this->set_last_error( COMM_RET_NOT_INITIALIZED );
        return COMM_RET_NOT_INITIALIZED;
    }
    CommMessage_t temp_msg;
    if ( xQueuePeek( this->rx_Queue, &temp_msg, 0 ) == pdTRUE ) {
        out_msg = temp_msg;
        this->set_last_error( COMM_RET_OK );
        return COMM_RET_OK;
    }
    this->set_last_error( COMM_RET_NO_DATA );
    return COMM_RET_NO_DATA;
}

size_t WiFiComm::available( ) const {
    if ( !this->rx_Queue ) {
        return 0;
    }
    return uxQueueMessagesWaiting( this->rx_Queue );
}

int32_t WiFiComm::get_rssi(){
    if (
        this->mode != WIFI_MODE_STA &&
        this->mode != WIFI_MODE_APSTA
    ) return 0;
    wifi_ap_record_t ap_info;
    esp_err_t ret =
        esp_wifi_sta_get_ap_info( &ap_info );
    if ( ret == ESP_OK) {
        return (int32_t)ap_info.rssi;
    } else {
        return 0;
    }
}

WiFiComm::~WiFiComm(){
    this->deinit();
}


bool wifi_scan_once(
    const char* target_ssid,
    wifi_ap_record_t *out_ap
) {
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        return false;
    }
    uint16_t num = 0;
    esp_err_t r = esp_wifi_scan_get_ap_num(&num);
    if (r != ESP_OK || num == 0) {
        return false;
    }
    wifi_ap_record_t *ap_list =
        (wifi_ap_record_t*) malloc(sizeof(wifi_ap_record_t) * num);
    if (!ap_list) {
        return false;
    }
    esp_wifi_scan_get_ap_records(&num, ap_list);
    bool found = false;
    for (int i = 0; i < (int)num; ++i) {
        if (target_ssid == NULL && !found) {
            if (out_ap) *out_ap = ap_list[i];
            found = true;
        } else if (target_ssid != NULL) {
            if (strncmp((char*)ap_list[i].ssid, target_ssid, sizeof(ap_list[i].ssid)) == 0) {
                if (out_ap) *out_ap = ap_list[i];
                found = true;
                break;
            }
        }
    }
    free(ap_list);
    return found;
}
