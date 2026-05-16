/**
 * @file    communication.cpp
 * @brief   Controlador de comunicacao (ProtocolComm).
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 18 de Novembro de 2025
 */


#include "communication.h"
#include "communication_utils.h"

ESP_EVENT_DEFINE_BASE( PROTO_EVENT_BASE );


CommRet_t ProtocolComm::init(){
    if (comm_base == nullptr) {
        ESP_LOGW("COMM BASE", "CommBase nao inicializado");
        return COMM_RET_NOT_INITIALIZED;
    }
    CommRet_t ret = comm_base->init();
    ESP_LOGI("COMM BASE", "CommBase inicializado - Ret: %s",
        comm_ret_to_str(ret));

    this->inflight_mutex = xSemaphoreCreateMutex();
    if ( this->callback_list_mutex == nullptr ) {
        this->callback_list_mutex = xSemaphoreCreateMutex();
    }
    memset( this->inflight_table, 0, sizeof(this->inflight_table));

    for ( size_t i = 0; i < this->MAX_INFLIGHT_MESSAGES; i++ ) {
        this->inflight_table[i].active = false;
        this->inflight_table[i].sem = xSemaphoreCreateBinary();
        this->inflight_table[i].tries_left = 0;
        this->inflight_table[i].seq = 0;
        this->inflight_table[i].timestamp_ms = 0;
        this->inflight_table[i].resp_ptr = nullptr;
        memset(
            &this->inflight_table[i].packet,
            0,
            sizeof(ProtoPacket_t)
        );
    }

    xTaskCreate(
        this->parser_Task,
        "ProtoRX_Task",
        PROTOCOL_PARSER_TASK_STACK_SIZE,
        this,
        PROTOCOL_PARSER_TASK_PRIORITY,
        NULL
    );

    xTaskCreate(
        this->trasmitter_Task,
        "ProtoTX_Task",
        PROTOCOL_TRANSMITTER_TASK_STACK_SIZE,
        this,
        PROTOCOL_TRANSMITTER_TASK_PRIORITY,
        NULL
    );
    return COMM_RET_OK;
}


void ProtocolComm::trasmitter_Handle(){
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10));

        xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);

        uint64_t now_ms = esp_timer_get_time() / 1000;

        for (size_t i = 0; i < MAX_INFLIGHT_MESSAGES; i++) {
            if (!this->inflight_table[i].active) continue;

            if (this->inflight_table[i].packet.header.flags & PROTO_FLAG_REQ_ACK) {

                if (now_ms - this->inflight_table[i].timestamp_ms > ACK_TIMEOUT_MS) {

                    if (this->inflight_table[i].tries_left > 0) {
                        this->inflight_table[i].timestamp_ms = now_ms;
                        this->inflight_table[i].tries_left--;

                        this->send_protocol_packet(
                            this->inflight_table[i].packet,
                            this->inflight_table[i].dest
                        );

                    } else {
                        ProtoComm::cleanup(this->inflight_table[i].packet);
                        memset(&this->inflight_table[i].packet, 0, sizeof(ProtoPacket_t));

                        if (this->inflight_table[i].sem) {
                            xQueueReset(this->inflight_table[i].sem);
                        }

                        this->inflight_table[i].active = false;
                    }
                }
            }
        }
        xSemaphoreGive(this->inflight_mutex);
    }
}


void ProtocolComm::parser_Handle(){
    ProtocolParsed_t *parsed_msg;
    ProtoPacket_t packet;
    CommMessage_t raw_msg;
    CommRet_t comm_ret;

    while ( true ){
        if ( this->comm_base == nullptr ) {
            vTaskDelay( pdMS_TO_TICKS( 10 ) );
            continue;
        }

        comm_ret = this->comm_base->receive( raw_msg );
        if ( comm_ret != COMM_RET_OK ) {
            vTaskDelay( pdMS_TO_TICKS( 10 ) );
            continue;
        }

        bool ret = ProtoComm::unpack(
            raw_msg.payload,
            raw_msg.length,
            packet
        );
        if ( !ret ) {
            ESP_LOGW("PROTO COMM", "Erro ao desempacotar mensagem recebida");
            continue;
        }

        parsed_msg = new ProtocolParsed_t();
        if ( parsed_msg == nullptr ) {
            ProtoComm::cleanup(packet);
            continue;
        }

        memset(parsed_msg, 0, sizeof(ProtocolParsed_t));

        parsed_msg->packet = packet;

        parsed_msg->package_len = PROTO_BASE_LEN + packet.header.payload_len;
        parsed_msg->seq_num = packet.header.seq;
        parsed_msg->src_addr = raw_msg.src;
        parsed_msg->dst_addr = raw_msg.dest;
        parsed_msg->timestamp_ms = esp_timer_get_time() / 1000;
        parsed_msg->expired.store(false);
        parsed_msg->user_ctx = nullptr;
        parsed_msg->next = nullptr;

        parsed_msg->retain_count.store(1);
        parsed_msg->auto_cleanup = true;

        if ( this->handle_incoming_packet( parsed_msg ) ){
            ProtocolParsed_t *ptr_parsed_msg = parsed_msg;
            esp_err_t err = esp_event_post_to(
                this->event_loop_handle,
                PROTO_EVENT_BASE,
                (int) parsed_msg->packet.header.cmd,
                &ptr_parsed_msg,
                sizeof(ProtocolParsed_t*),
                pdMS_TO_TICKS(10)
            );
            if ( err != ESP_OK ) {
                ProtoComm::cleanup( parsed_msg->packet );
                delete parsed_msg;
                continue;
            }
        }
    }
}


bool ProtocolComm::handle_incoming_packet(
    ProtocolParsed_t *parsed_msg
){
    if (
        parsed_msg->packet.header.id != this->id_device &&
        parsed_msg->packet.header.id != PROTO_ID_BROADCAST
    ) {
        ProtoComm::cleanup( parsed_msg->packet );
        delete parsed_msg;
        return false;
    }

    const uint8_t  flags = parsed_msg->packet.header.flags;

    if ( flags == PROTO_FLAG_NOP ){
        return true;
    }

    else if ( flags & PROTO_FLAG_RESERVED ){
        ProtoComm::cleanup( parsed_msg->packet );
        delete parsed_msg;
        return false;
    }

    const uint8_t  cmd   = parsed_msg->packet.header.cmd;
    const uint16_t seq   = parsed_msg->packet.header.seq;

    if ( flags & PROTO_FLAG_RESP ){
        bool ownership_transferred = false;

        xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);
        InflightEntry_t *entry = this->lookup_inflight_message( seq );

        if ( entry != nullptr ) {
            if ( entry->resp_ptr == nullptr ) {
                entry->resp_ptr = parsed_msg;
                ownership_transferred = true;

                if ( entry->sem ) {
                    xSemaphoreGive( entry->sem );
                }
            }
        }
        xSemaphoreGive(this->inflight_mutex);

        if (!ownership_transferred) {
            ProtocolComm::free_parsed( parsed_msg );
        }
        return false;
    }

    else if ( flags & PROTO_FLAG_REQ_RESP ){
        return true;
    }

    else if ( flags & PROTO_FLAG_ACK ){
        xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);
        InflightEntry_t *entry = this->lookup_inflight_message( seq );

        if ( entry != nullptr ) {
            if ( entry->packet.header.flags & PROTO_FLAG_REQ_ACK ) {
                ProtoComm::cleanup(entry->packet);
                memset(&entry->packet, 0, sizeof(ProtoPacket_t));
                entry->active = false;
            }
        }
        xSemaphoreGive(this->inflight_mutex);

        ProtocolComm::free_parsed( parsed_msg );
        return false;
    }

    else if ( flags & PROTO_FLAG_REQ_ACK ){
        ProtoPacket_t ack_packet;
        memset(&ack_packet, 0, sizeof(ack_packet));

        ack_packet.header.flags = PROTO_FLAG_ACK;
        ack_packet.header.id = PROTO_ID_BROADCAST;
        ack_packet.header.cmd = cmd;
        ack_packet.header.seq = seq;
        ack_packet.header.payload_len = 0;

        this->send_protocol_packet(ack_packet, parsed_msg->src_addr);
        return true;
    }

    ProtoComm::cleanup( parsed_msg->packet );
    delete parsed_msg;
    return false;
}


void ProtocolComm::clear_inflight_slot( uint8_t idx ) {
    if (idx >= MAX_INFLIGHT_MESSAGES) return;

    if (this->inflight_table[idx].timeout_timer) {
        esp_timer_stop(this->inflight_table[idx].timeout_timer);
        esp_timer_delete(this->inflight_table[idx].timeout_timer);
        this->inflight_table[idx].timeout_timer = nullptr;
    }

    ProtoComm::cleanup(this->inflight_table[idx].packet);
    memset(&this->inflight_table[idx].packet, 0, sizeof(ProtoPacket_t));

    if (this->inflight_table[idx].resp_ptr) {
        ProtoComm::cleanup(this->inflight_table[idx].resp_ptr->packet);
        free(this->inflight_table[idx].resp_ptr);
        this->inflight_table[idx].resp_ptr = nullptr;
    }

    if (this->inflight_table[idx].sem) {
        xQueueReset(this->inflight_table[idx].sem);
    }

    this->inflight_table[idx].active = false;
    this->inflight_table[idx].tries_left = 0;
    this->inflight_table[idx].seq = 0;
    this->inflight_table[idx].timestamp_ms = 0;
    memset(&this->inflight_table[idx].dest, 0, sizeof(CommAddr_t));
}


InflightEntry_t* ProtocolComm::lookup_inflight_message( int16_t seq_num ){
    for ( size_t i = 0; i < MAX_INFLIGHT_MESSAGES; i++ ) {
        if (
            this->inflight_table[i].active &&
            this->inflight_table[i].seq == seq_num
        ) {
            return &this->inflight_table[i];
        }
    }
    return nullptr;
}


int16_t ProtocolComm::allocate_inflight_slot(
    const InflightEntry_t &entry
) {
    xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);
    for ( size_t i = 0; i < MAX_INFLIGHT_MESSAGES; i++ ) {
        if ( !this->inflight_table[i].active ) {
            SemaphoreHandle_t slot_sem = this->inflight_table[i].sem;

            this->inflight_table[i] = entry;

            this->inflight_table[i].sem = slot_sem;

            if (
                entry.packet.payload &&
                entry.packet.header.payload_len > 0
            ) {
                size_t len = entry.packet.header.payload_len;
                this->inflight_table[i].packet.payload = new uint8_t[len];
                memcpy(
                    this->inflight_table[i].packet.payload,
                    entry.packet.payload,
                    len
                );
            }
            this->inflight_table[i].active = true;
            xSemaphoreGive(this->inflight_mutex);
            return (int16_t)i;
        }
    }
    xSemaphoreGive(this->inflight_mutex);
    return -1;
}


esp_err_t ProtocolComm::create_event_loop(){
    esp_err_t err =
        esp_event_loop_create(
            &this->event_loop_config_args,
            &this->event_loop_handle
        );
    if ( err != ESP_OK ) {
        ESP_LOGE("PROTO COMM", "Falha ao criar event loop: %s",
            esp_err_to_name(err));
        return err;
    }
    return err;
}

esp_err_t ProtocolComm::register_event_loop_callback(
    esp_event_handler_t callback,
    void *callbakc_arg
){
    return this->register_event_loop_callback(
        callback,
        ESP_EVENT_ANY_ID,
        callbakc_arg
    );
}

esp_err_t ProtocolComm::register_event_loop_callback(
    esp_event_handler_t callback,
    int8_t id_event,
    void *callbakc_arg
){
    if ( callback == nullptr ) {
        return ESP_ERR_INVALID_ARG;
    }

    if ( this->event_loop_handle == nullptr ) {
        ESP_LOGW("PROTO COMM",
            "Event loop nao criado. Chame create_event_loop() antes");
        return ESP_ERR_INVALID_STATE;
    }

    if ( this->callback_list_mutex == nullptr ) {
        this->callback_list_mutex = xSemaphoreCreateMutex();
        if ( this->callback_list_mutex == nullptr ) {
            return ESP_ERR_NO_MEM;
        }
    }

    xSemaphoreTake(this->callback_list_mutex, portMAX_DELAY);

    if ( this->event_loop_callback_instance == nullptr ) {
        esp_err_t err = esp_event_handler_instance_register_with(
            this->event_loop_handle,
            PROTO_EVENT_BASE,
            ESP_EVENT_ANY_ID,
            &ProtocolComm::dispatch_event_loop_handler,
            this,
            &this->event_loop_callback_instance
        );
        if ( err != ESP_OK ) {
            xSemaphoreGive(this->callback_list_mutex);
            return err;
        }
    }

    EventCallbackNode *node = new EventCallbackNode;
    node->callback = callback;
    node->arg = callbakc_arg;
    node->event_id = id_event;
    node->next = this->callback_list_head;
    this->callback_list_head = node;

    xSemaphoreGive(this->callback_list_mutex);

    return ESP_OK;
}


esp_err_t ProtocolComm::unregister_event_loop_callback(){
    return this->unregister_event_loop_callback( ESP_EVENT_ANY_ID );
}

esp_err_t ProtocolComm::unregister_event_loop_callback(
    int8_t id_event
){
    if ( this->callback_list_mutex == nullptr ) {
        return ESP_OK;
    }

    xSemaphoreTake(this->callback_list_mutex, portMAX_DELAY);

    EventCallbackNode *current = this->callback_list_head;
    EventCallbackNode *prev = nullptr;

    while ( current != nullptr ) {
        bool remove = false;
        if ( id_event == ESP_EVENT_ANY_ID ) {
            remove = true;
        } else if ( current->event_id == id_event ) {
            remove = true;
        }

        if ( remove ) {
            EventCallbackNode *next = current->next;
            if ( prev == nullptr ) {
                this->callback_list_head = next;
            } else {
                prev->next = next;
            }
            delete current;
            current = next;
        } else {
            prev = current;
            current = current->next;
        }
    }

    if (
        this->callback_list_head == nullptr &&
        this->event_loop_callback_instance != nullptr
    ) {
        esp_event_handler_instance_unregister_with(
            this->event_loop_handle,
            PROTO_EVENT_BASE,
            ESP_EVENT_ANY_ID,
            this->event_loop_callback_instance
        );
        this->event_loop_callback_instance = nullptr;
    }

    xSemaphoreGive(this->callback_list_mutex);

    return ESP_OK;
}



CommRet_t ProtocolComm::send_reliable(
    uint8_t cmd,
    const void *payload,
    size_t len,
    const CommAddr_t &dest
){
    uint16_t seq = this->seq_counter.fetch_add(1);

    ProtoPacket_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.header.flags = PROTO_FLAG_REQ_ACK;
    packet.header.id = PROTO_ID_BROADCAST;
    packet.header.cmd = cmd;
    packet.header.seq = seq;
    packet.header.payload_len = len;
    packet.payload = (uint8_t*)payload;

    InflightEntry_t entry;
    memset(&entry, 0, sizeof(entry));
    entry.active = true;
    entry.seq = seq;
    entry.dest = dest;
    entry.timestamp_ms = esp_timer_get_time() / 1000;
    entry.tries_left = MAX_RETRIES;
    entry.packet = packet;

    int16_t slot = this->allocate_inflight_slot(entry);
    if (slot < 0) {
        return COMM_RET_NO_MEMORY;
    }

    CommRet_t ret = this->send_protocol_packet(packet, dest);
    (void)ret;

    return COMM_RET_OK;
}


void ProtocolComm::dispatch_event_loop_handler(
    void* handler_arg,
    esp_event_base_t base,
    int32_t id,
    void* event_data
){
    ProtocolComm *comm = static_cast<ProtocolComm*>( handler_arg );

    if ( event_data == nullptr ) {
        return;
    }

    ProtocolParsed_t *parsed_msg = *(ProtocolParsed_t**)event_data;
    if ( parsed_msg == nullptr ) {
        return;
    }

    if ( comm->callback_list_mutex != nullptr ) {
        xSemaphoreTake(comm->callback_list_mutex, portMAX_DELAY);

        EventCallbackNode *node = comm->callback_list_head;
        while( node != nullptr ) {
            if (
                node->event_id == ESP_EVENT_ANY_ID ||
                node->event_id == id
            ) {
                if ( node->callback ) {
                    node->callback(
                        node->arg,
                        base,
                        id,
                        (void*)event_data
                    );
                }
            }
            node = node->next;
        }

        xSemaphoreGive(comm->callback_list_mutex);
    }

    ProtocolComm::free_parsed( parsed_msg );
}


CommRet_t ProtocolComm::send_protocol_packet(
    ProtoPacket_t &packet,
    const CommAddr_t &dest
){
    size_t total_len = PROTO_BASE_LEN + packet.header.payload_len;
    uint8_t *buffer = new uint8_t[total_len];
    if (!buffer) return COMM_RET_IO_ERROR;

    size_t written = ProtoComm::pack(packet, buffer, total_len);
    if (written == 0) {
        delete[] buffer;
        return COMM_RET_IO_ERROR;
    }

    CommMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.dest = dest;

    if (written > sizeof(msg.payload)) {
        delete[] buffer;
        return COMM_RET_UNKNOWN_ERROR;
    }

    memcpy(msg.payload, buffer, written);
    msg.length = written;

    CommRet_t ret = this->send_to(msg, dest);

    delete[] buffer;
    return ret;
}

CommRet_t ProtocolComm::send_reply(
    ProtocolParsed_t *req,
    const void *payload,
    size_t len
){
    if (!req) return COMM_RET_IO_ERROR;

    ProtoPacket_t resp_packet;
    memset(&resp_packet, 0, sizeof(resp_packet));

    resp_packet.header.flags = PROTO_FLAG_RESP;
    resp_packet.header.id = req->packet.header.id;
    resp_packet.header.cmd = req->packet.header.cmd;
    resp_packet.header.seq = req->seq_num;
    resp_packet.header.payload_len = len;
    resp_packet.payload = (uint8_t*)payload;

    return this->send_protocol_packet(resp_packet, req->src_addr);
}

CommRet_t ProtocolComm::send_request_sync(
    uint8_t cmd,
    const void *payload,
    size_t len,
    const CommAddr_t &dest,
    ProtocolParsed_t **response,
    uint32_t timeout_ms
){
    if (response) *response = nullptr;

    uint16_t seq = this->seq_counter.fetch_add(1);

    ProtoPacket_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.header.flags = PROTO_FLAG_REQ_RESP;
    packet.header.id = PROTO_ID_BROADCAST;
    packet.header.cmd = cmd;
    packet.header.seq = seq;
    packet.header.payload_len = len;
    packet.payload = (uint8_t*)payload;

    InflightEntry_t entry;
    memset(&entry, 0, sizeof(entry));
    entry.active = true;
    entry.seq = seq;
    entry.dest = dest;
    entry.timestamp_ms = esp_timer_get_time() / 1000;
    entry.packet = packet;

    int16_t slot = this->allocate_inflight_slot(entry);
    if (slot < 0) {
        return COMM_RET_IO_ERROR;
    }

    CommRet_t ret = this->send_protocol_packet(packet, dest);
    if (ret != COMM_RET_OK) {
        this->clear_inflight_slot(slot);
        return ret;
    }

    xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);
    SemaphoreHandle_t sem = this->inflight_table[slot].sem;
    xSemaphoreGive(this->inflight_mutex);

    xQueueReset(sem);

    if (xSemaphoreTake(sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        xSemaphoreTake(this->inflight_mutex, portMAX_DELAY);
        if (this->inflight_table[slot].resp_ptr) {
            if (response) {
                *response = this->inflight_table[slot].resp_ptr;
                this->inflight_table[slot].resp_ptr = nullptr;
            } else {
                ProtocolComm::free_parsed(
                    this->inflight_table[slot].resp_ptr
                );
                this->inflight_table[slot].resp_ptr = nullptr;
            }
        }
        xSemaphoreGive(this->inflight_mutex);

        this->clear_inflight_slot(slot);
        return COMM_RET_OK;
    } else {
        this->clear_inflight_slot(slot);
        return COMM_RET_TIMEOUT;
    }
}
