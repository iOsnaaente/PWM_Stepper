/**
 * @file    communication_utils.h
 * @brief   Utilitarios para o controlador de comunicacao.
 * @author  Bruno G. F. Sampaio
 * @date    Criado em 18 de Novembro de 2025
 */

#pragma once

#include "board_config.h"

#include "interfaces/proto_base.h"
#include "interfaces/comm_base.h"

#include <functional>
#include <cstdint>
#include <cstddef>
#include <atomic>

#define PROTOCOL_DEFAULT_MAX_RETRIES    5
#define PROTOCOL_DEFAULT_TIMEOUT_MS     1000

#define PROTOCOL_TRANSMITTER_TASK_STACK_SIZE    1024*4
#define PROTOCOL_TRANSMITTER_TASK_PRIORITY      (tskIDLE_PRIORITY + 10)

#define PROTOCOL_PARSER_TASK_STACK_SIZE         1024*4
#define PROTOCOL_PARSER_TASK_PRIORITY           (tskIDLE_PRIORITY + 10)


typedef struct ProtocolParsed {
    ProtoPacket_t            packet;
    uint16_t                 package_len;
    uint16_t                 seq_num;
    CommAddr_t               src_addr;
    CommAddr_t               dst_addr;
    uint64_t                 timestamp_ms;
    std::atomic<bool>        expired;
    std::atomic<int>         retain_count{0};
    bool                     auto_cleanup = true;
    void                    *user_ctx;
    struct ProtocolParsed   *next;
} ProtocolParsed_t;


typedef struct InflightEntry {
    bool                active = false;
    ProtoPacket_t       packet;
    CommAddr_t          dest;
    int16_t             seq = 0;
    uint8_t             tries_left = 0;
    uint64_t            timestamp_ms = 0;
    esp_timer_handle_t  timeout_timer;
    SemaphoreHandle_t   sem = nullptr;
    ProtocolParsed_t   *resp_ptr = nullptr;
} InflightEntry_t;
