/**
 * @file    comm_base_utils.h
 * @brief   Tipos auxiliares para a interface base de comunicação.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Criado em 12 de Outubro de 2025
 */

#pragma once

#include "stdint.h"
#include "stddef.h"
#include "stdlib.h"
#include "stdio.h"

typedef enum : uint8_t {
    COMM_TYPE_NONE = 0,
    COMM_TYPE_NRF24,
    COMM_TYPE_ESP_NOW,
    COMM_TYPE_BLUETOOTH,
    COMM_TYPE_WIFI,
    COMM_TYPE_UROS,
} CommType_t;

static inline const char* comm_type_to_str(CommType_t type) {
    switch (type) {
        case COMM_TYPE_NONE:      return "COMM_TYPE_NONE";
        case COMM_TYPE_NRF24:     return "COMM_TYPE_NRF24";
        case COMM_TYPE_ESP_NOW:   return "COMM_TYPE_ESP_NOW";
        case COMM_TYPE_BLUETOOTH: return "COMM_TYPE_BLUETOOTH";
        case COMM_TYPE_WIFI:      return "COMM_TYPE_WIFI";
        case COMM_TYPE_UROS:      return "COMM_TYPE_UROS";
        default:                  return "COMM_TYPE_UNKNOWN";
    }
}


typedef enum : uint8_t {
    COMM_MSG_TYPE_UNKNOWN = 0,
    COMM_MSG_TYPE_DATA,
    COMM_MSG_TYPE_CONFIG,
    COMM_MSG_TYPE_ODOMETRY,
    COMM_MSG_TYPE_HEARTBEAT,
    COMM_MSG_TYPE_ACK,
    COMM_MSG_TYPE_ERROR,
    COMM_MSG_TYPE_RAW,
} CommMessageType_t;

static inline const char* comm_msg_type_to_str(CommMessageType_t type) {
    switch (type) {
        case COMM_MSG_TYPE_UNKNOWN:    return "UNKNOWN";
        case COMM_MSG_TYPE_DATA:       return "DATA";
        case COMM_MSG_TYPE_CONFIG:     return "CONFIG";
        case COMM_MSG_TYPE_ODOMETRY:   return "ODOMETRY";
        case COMM_MSG_TYPE_HEARTBEAT:  return "HEARTBEAT";
        case COMM_MSG_TYPE_ACK:        return "ACK";
        case COMM_MSG_TYPE_ERROR:      return "ERROR";
        case COMM_MSG_TYPE_RAW:        return "RAW";
        default:                       return "INVALID";
    }
}


typedef enum : int8_t {
    COMM_RET_OK = 0,
    COMM_RET_TIMEOUT,
    COMM_RET_NO_MEMORY,
    COMM_RET_INVALID_ARG,
    COMM_RET_NOT_INITIALIZED,
    COMM_RET_BUSY,
    COMM_RET_NO_DATA,
    COMM_RET_IO_ERROR,
    COMM_RET_NOT_IMPLEMENTED,
    COMM_RET_UNKNOWN_ERROR = -128
} CommRet_t;

static inline const char* comm_ret_to_str(CommRet_t code) {
    switch (code) {
        case COMM_RET_OK:              return "OK";
        case COMM_RET_TIMEOUT:         return "TIMEOUT";
        case COMM_RET_NO_MEMORY:       return "NO_MEMORY";
        case COMM_RET_INVALID_ARG:     return "INVALID_ARG";
        case COMM_RET_NOT_INITIALIZED: return "NOT_INITIALIZED";
        case COMM_RET_BUSY:            return "BUSY";
        case COMM_RET_NO_DATA:         return "NO_DATA";
        case COMM_RET_IO_ERROR:        return "IO_ERROR";
        case COMM_RET_NOT_IMPLEMENTED: return "NOT_IMPLEMENTED";
        case COMM_RET_UNKNOWN_ERROR:   return "UNKNOWN_ERROR";
        default:                       return "INVALID";
    }
}


typedef enum : uint8_t {
    COMM_STATE_UNINITIALIZED = 0,
    COMM_STATE_IDLE,
    COMM_STATE_RUNNING,
    COMM_STATE_STOPPED,
    COMM_STATE_SENDING,
    COMM_STATE_RECEIVING,
    COMM_STATE_FAULT
} CommState_t;

static inline const char* comm_state_to_str(CommState_t state) {
    switch (state) {
        case COMM_STATE_UNINITIALIZED: return "UNINITIALIZED";
        case COMM_STATE_IDLE:          return "IDLE";
        case COMM_STATE_RUNNING:       return "RUNNING";
        case COMM_STATE_STOPPED:       return "STOPPED";
        case COMM_STATE_SENDING:       return "SENDING";
        case COMM_STATE_RECEIVING:     return "RECEIVING";
        case COMM_STATE_FAULT:         return "FAULT";
        default:                       return "UNKNOWN";
    }
}

static constexpr size_t COMM_MAX_PAYLOAD = 64;


typedef struct __attribute__((packed)) {
    CommType_t     type;
    uint8_t        data[16];
    uint8_t        len;
} CommAddr_t;


typedef struct __attribute__((packed)) {
    uint32_t          msg_id;
    CommMessageType_t type;
    uint8_t           payload[COMM_MAX_PAYLOAD];
    uint32_t          length;
    CommAddr_t        src;
    CommAddr_t        dest;
    uint64_t          timestamp;
} CommMessage_t;


typedef struct  __attribute__((packed)) {
    uint32_t   peer_id;
    CommAddr_t addr;
} CommPeer_t;
