/**
 * @file    communication.h
 * @brief   Controlador de comunicacao (ProtocolComm).
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 18 de Novembro de 2025
 */

#pragma once


#include "communication_utils.h"


ESP_EVENT_DECLARE_BASE( PROTO_EVENT_BASE );


#define PROTO_EVENT_LOOP_CONFIG_DEFAULT {       \
    .queue_size = 32,                           \
    .task_name = "ProtoEventLoop",              \
    .task_priority = uxTaskPriorityGet(NULL),   \
    .task_stack_size = 4096,                    \
    .task_core_id = tskNO_AFFINITY              \
};


class ProtocolComm {
    private:
        const char *description;
        CommBase *comm_base;
        int8_t id_device;

        std::atomic<uint16_t> seq_counter{0};

        static constexpr uint8_t MAX_INFLIGHT_MESSAGES = 64;

        static constexpr uint8_t  MAX_RETRIES = 5;
        static constexpr uint32_t ACK_TIMEOUT_MS = 200;

        InflightEntry_t          inflight_table[MAX_INFLIGHT_MESSAGES];
        SemaphoreHandle_t        inflight_mutex;

        esp_event_loop_args_t event_loop_config_args =
            PROTO_EVENT_LOOP_CONFIG_DEFAULT;
        esp_event_loop_handle_t event_loop_handle;

        esp_event_handler_instance_t
            event_loop_callback_instance = nullptr;

        struct EventCallbackNode {
            esp_event_handler_t callback;
            void *arg;
            int32_t event_id;
            EventCallbackNode *next;
        };

        EventCallbackNode *callback_list_head = nullptr;
        SemaphoreHandle_t callback_list_mutex = nullptr;


        static void trasmitter_Task( void * pvParameters ){
            ProtocolComm *comm =
                static_cast<ProtocolComm*>( pvParameters );
            comm->trasmitter_Handle();
        }
        void trasmitter_Handle();

        static void parser_Task( void * pvParameters ){
            ProtocolComm *comm =
                static_cast<ProtocolComm*>( pvParameters );
            comm->parser_Handle();
        }
        void parser_Handle();

        bool handle_incoming_packet(
            ProtocolParsed_t *parsed_msg
        );

        static void dispatch_event_loop_handler(
            void *handler_arg,
            esp_event_base_t base,
            int32_t id,
            void *event_data
        );

    public:
        esp_err_t create_event_loop();

        esp_err_t register_event_loop_callback(
            esp_event_handler_t callback,
            int8_t id_event,
            void *callbakc_arg
        );
        esp_err_t register_event_loop_callback(
            esp_event_handler_t callback,
            void *callbakc_arg
        );

        esp_err_t unregister_event_loop_callback( int8_t id_event );
        esp_err_t unregister_event_loop_callback();


        ProtocolComm(
            const char *description = "ProtocolComm",
            uint8_t id_device = PROTO_ID_BROADCAST,
            CommBase *base = nullptr
        ) : description(description),
            comm_base(base),
            id_device(id_device),
            inflight_mutex(nullptr),
            event_loop_handle(nullptr)
        {
            if ( comm_base == nullptr ) {
                ESP_LOGW("WIRELESS COMM",
                    "CommBase nao fornecido, comunicacao nao inicializada");
            }
        }

        ~ProtocolComm(){
            this->stop();
            this->deinit();
        }

        CommRet_t init();

        CommRet_t deinit(){
            CommRet_t ret = COMM_RET_OK;
            if (comm_base != nullptr) {
                ret = comm_base->deinit();
            }
            return ret;
        }

        CommRet_t start(){
            if (comm_base != nullptr) {
                return comm_base->start();
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        CommRet_t stop(){
            if (comm_base != nullptr) {
                return comm_base->stop();
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        CommRet_t send(
            const CommMessage_t &msg,
            uint32_t timeout_ms = 0
        ){
            if (comm_base != nullptr) {
                return comm_base->send(msg, timeout_ms);
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        CommRet_t send_to(
            const CommMessage_t &msg,
            const CommAddr_t &addr,
            uint32_t timeout_ms = 0
        ){
            if (comm_base != nullptr) {
                return comm_base->send_to(msg, addr, timeout_ms);
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        CommRet_t send_reply(
            ProtocolParsed_t *req,
            const void *payload,
            size_t len
        );

        CommRet_t send_request_sync(
            uint8_t cmd,
            const void *payload,
            size_t len,
            const CommAddr_t &dest,
            ProtocolParsed_t **response,
            uint32_t timeout_ms = 1000
        );

        CommRet_t send_reliable(
            uint8_t cmd,
            const void *payload,
            size_t len,
            const CommAddr_t &dest
        );

    private:
        CommRet_t send_protocol_packet(
            ProtoPacket_t &packet,
            const CommAddr_t &dest
        );

    public:
        CommRet_t receive(
            CommMessage_t &out_msg,
            uint32_t timeout_ms = 0
        ){
            if (comm_base != nullptr) {
                return comm_base->receive(out_msg, timeout_ms);
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        CommRet_t receive_from(
            CommMessage_t &out_msg,
            CommAddr_t &addr,
            uint32_t timeout_ms = 0
        ){
            out_msg.dest = addr;
            return this->receive(out_msg, timeout_ms);
        }

        CommRet_t peek(CommMessage_t &out_msg) {
            if (comm_base != nullptr) {
                return comm_base->peek(out_msg);
            }
            return COMM_RET_NOT_INITIALIZED;
        }

        size_t available() const {
            if (comm_base != nullptr) {
                return comm_base->available();
            }
            return 0;
        }

        CommState_t get_state() const {
            if (comm_base != nullptr) {
                return comm_base->get_state();
            }
            return COMM_STATE_UNINITIALIZED;
        }

        void set_comm_base( CommBase *base ) {
            this->comm_base = base;
        }
        CommBase* get_comm_base() const {
            return this->comm_base;
        }
        void set_id_device( uint8_t id ) {
            this->id_device = id;
        }
        uint8_t get_id_device() const {
            return this->id_device;
        }

        static bool retain_parsed( ProtocolParsed_t* parsed_msg ) {
            if ( parsed_msg == nullptr ) {
                return false;
            }
            parsed_msg->retain_count.fetch_add(
                1, std::memory_order_acq_rel
            );
            return true;
        }

        static void release_parsed( ProtocolParsed_t* parsed_msg ) {
            if ( parsed_msg == nullptr ) {
                return;
            }
            int prev_count =
                parsed_msg->retain_count.fetch_sub(
                    1,
                    std::memory_order_acq_rel
                );

            if ( prev_count <= 0 ) {
                parsed_msg->retain_count.store(0);
                if ( parsed_msg->auto_cleanup ) {
                    ProtoComm::cleanup( parsed_msg->packet );
                    delete parsed_msg;
                }
            }
        }

        static void free_parsed( ProtocolParsed_t* parsed_msg ) {
            if ( parsed_msg == nullptr ) {
                return;
            }
            ProtoComm::cleanup( parsed_msg->packet );
            delete parsed_msg;
        }


    private:
        void clear_inflight_slot( uint8_t idx );
        InflightEntry_t *lookup_inflight_message( int16_t seq_num );
        int16_t allocate_inflight_slot( const InflightEntry_t &entry );
};
