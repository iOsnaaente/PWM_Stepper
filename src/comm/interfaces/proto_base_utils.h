/**
 * @file    proto_base_utils.h
 * @brief   Utilitarios do protocolo de comunicacao.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Criado em 12 de Outubro de 2025
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>


typedef enum : uint8_t {
    PROTO_FLAG_NOP       = 0b00000000,
    PROTO_FLAG_ACK       = 0b00000001,
    PROTO_FLAG_REQ_ACK   = 0b00000010,
    PROTO_FLAG_PERSIST   = 0b00000100,
    PROTO_FLAG_RESP      = 0b00001000,
    PROTO_FLAG_REQ_RESP  = 0b00010000,
    PROTO_FLAG_RESERVED  = 0b11100000,
} ProtoFlag_t;

static inline const char* proto_flag_to_name( ProtoFlag_t flag ) {
    switch (flag) {
        case PROTO_FLAG_NOP:        return "PROTO_FLAG_NOP";
        case PROTO_FLAG_ACK:        return "PROTO_FLAG_ACK";
        case PROTO_FLAG_REQ_ACK:    return "PROTO_FLAG_REQ_ACK";
        case PROTO_FLAG_PERSIST:    return "PROTO_FLAG_PERSIST";
        case PROTO_FLAG_RESP:       return "PROTO_FLAG_RESP";
        case PROTO_FLAG_REQ_RESP:   return "PROTO_FLAG_REQ_RESP";
        case PROTO_FLAG_RESERVED:   return "PROTO_FLAG_RESERVED";
        default:                    return "PROTO_FLAG_INVALID";
    }
}


typedef enum : uint8_t {
    PROTO_CMD_NOP                   = 0x00,
    PROTO_CMD_ACK                   = 0x01,
    PROTO_CMD_SET_ID_DEVICE         = 0x02,
    PROTO_CMD_GET_ID_DEVICE         = 0x03,
    PROTO_CMD_STOP                  = 0x10,
    PROTO_CMD_SET_TARGET_VEL        = 0x11,
    PROTO_CMD_SET_MMPS_VEL          = 0x12,
    PROTO_CMD_SET_MPS_VEL           = 0x13,
    PROTO_CMD_SET_RPM_VEL           = 0x14,
    PROTO_CMD_SET_RADS_VEL          = 0x15,
    PROTO_CMD_SET_VECTOR_VEL        = 0x16,
    PROTO_CMD_SET_TARGET_XY         = 0x17,
    PROTO_CMD_SET_TRAJECTORY_TARGET = 0x18,
    PROTO_CMD_SET_TRAJECTORY_MMPS   = 0x19,
    PROTO_CMD_SET_TRAJECTORY_MPS    = 0x1A,
    PROTO_CMD_SET_TRAJECTORY_RPMS   = 0x1B,
    PROTO_CMD_SET_TRAJECTORY_RADS   = 0x1C,
    PROTO_CMD_SET_TRAJECTORY_VECTOR = 0x1D,
    PROTO_CMD_SET_TRAJECTORY_XY     = 0x1E,
    PROTO_CMD_SET_VEL_PARAMS        = 0x1F,
    PROTO_CMD_CONFIG_PID            = 0x20,
    PROTO_CMD_CONFIG_PID_KP         = 0x21,
    PROTO_CMD_CONFIG_PID_KI         = 0x22,
    PROTO_CMD_CONFIG_PID_KD         = 0x23,
    PROTO_CMD_CONFIG_PID_DT         = 0x24,
    PROTO_CMD_TEST_STEP             = 0x25,
    PROTO_CMD_TEST_RAMP             = 0x26,
    PROTO_CMD_TEST_SINE             = 0x27,
    PROTO_CMD_CONFIG_PID_RESET      = 0x2F,
    PROTO_CMD_CONFIG_READ           = 0x30,
    PROTO_CMD_CONFIG_WRITE          = 0x31,
    PROTO_CMD_CONFIG_LIST           = 0x32,
    PROTO_CMD_NVS_COMMIT            = 0x33,
    PROTO_CMD_TELEMETRY             = 0x40,
    PROTO_CMD_HEARTBEAT             = 0x41,
    PROTO_CMD_PING                  = 0x42,
    PROTO_CMD_REQ_PING              = 0x43,
    PROTO_CMD_REQ_ACCEL_DATA        = 0x44,
    PROTO_CMD_DEBUG_TEXT            = 0xA0,
    PROTO_CMD_TEXT                  = 0xA1,
    PROTO_CMD_BUFFER                = 0xA2,
    PROTO_CMD_JSON                  = 0xA3,
    PROTO_CMD_ERROR_UNKNOW          = 0xF0,
    PROTO_CMD_ERROR_HEADER          = 0xF1,
    PROTO_CMD_ERROR_RESERVED        = 0xF2,
    PROTO_CMD_ERROR_PAYLOAD         = 0xF3,
    PROTO_CMD_ERROR_COMMAND         = 0xF4,
    PROTO_CMD_ERROR_CRC             = 0xF5,
    PROTO_CMD_UNKNOWN               = 0xFF,
} ProtoCommand_t;

static inline const char* proto_command_to_name( ProtoCommand_t comm ) {
    switch (comm) {
        case PROTO_CMD_NOP:            return "PROTO_CMD_NOP";
        case PROTO_CMD_SET_TARGET_VEL: return "PROTO_CMD_SET_TARGET_VEL";
        case PROTO_CMD_SET_TARGET_XY:  return "PROTO_CMD_SET_TARGET_XY";
        case PROTO_CMD_STOP:           return "PROTO_CMD_STOP";
        case PROTO_CMD_CONFIG_PID:     return "PROTO_CMD_CONFIG_PID";
        case PROTO_CMD_TEST_STEP:      return "PROTO_CMD_TEST_STEP";
        case PROTO_CMD_TEST_RAMP:      return "PROTO_CMD_TEST_RAMP";
        case PROTO_CMD_TEST_SINE:      return "PROTO_CMD_TEST_SINE";
        case PROTO_CMD_CONFIG_READ:    return "PROTO_CMD_CONFIG_READ";
        case PROTO_CMD_CONFIG_WRITE:   return "PROTO_CMD_CONFIG_WRITE";
        case PROTO_CMD_CONFIG_LIST:    return "PROTO_CMD_CONFIG_LIST";
        case PROTO_CMD_NVS_COMMIT:     return "PROTO_CMD_NVS_COMMIT";
        case PROTO_CMD_TELEMETRY:      return "PROTO_CMD_TELEMETRY";
        case PROTO_CMD_HEARTBEAT:      return "PROTO_CMD_HEARTBEAT";
        case PROTO_CMD_PING:           return "PROTO_CMD_PING";
        case PROTO_CMD_REQ_PING:       return "PROTO_CMD_REQ_PING";
        case PROTO_CMD_REQ_ACCEL_DATA: return "PROTO_CMD_REQ_ACCEL_DATA";
        case PROTO_CMD_ACK:            return "PROTO_CMD_ACK";
        case PROTO_CMD_ERROR_UNKNOW:   return "PROTO_CMD_ERROR_UNKNOW";
        case PROTO_CMD_ERROR_HEADER:   return "PROTO_CMD_ERROR_HEADER";
        case PROTO_CMD_ERROR_RESERVED: return "PROTO_CMD_ERROR_RESERVED";
        case PROTO_CMD_ERROR_PAYLOAD:  return "PROTO_CMD_ERROR_PAYLOAD";
        case PROTO_CMD_ERROR_CRC:      return "PROTO_CMD_ERROR_CRC";
        case PROTO_CMD_DEBUG_TEXT:     return "PROTO_CMD_DEBUG_TEXT";
        case PROTO_CMD_JSON:           return "PROTO_CMD_JSON";
        case PROTO_CMD_UNKNOWN:        return "PROTO_CMD_UNKNOWN";
        default:                       return "PROTO_CMD_INVALID";
    }
}


typedef struct __attribute__((packed)) {
    uint8_t   flags;
    uint8_t   id;
    uint8_t   cmd;
    uint16_t  seq;
    uint8_t   payload_len;
} ProtoHeader_t;


typedef struct __attribute__((packed)) {
    ProtoHeader_t header;
    uint8_t *payload;
    uint16_t crc;
} ProtoPacket_t;


static constexpr uint8_t PROTO_MAX_PAYLOAD_LEN = 255;
static constexpr uint8_t PROTO_ID_BROADCAST = 0xFF;
static constexpr uint8_t PROTO_CRC_LEN = 0x02;

static constexpr uint8_t PROTO_HEADER_LEN = sizeof(ProtoHeader_t);
static constexpr uint8_t PROTO_BASE_LEN = PROTO_HEADER_LEN + PROTO_CRC_LEN;


static inline ProtoPacket_t proto_ack( uint8_t id, uint16_t seq_num ) {
    ProtoPacket_t packet;
    packet.header.flags = PROTO_FLAG_ACK | PROTO_FLAG_RESP;
    packet.header.id = id;
    packet.header.cmd = PROTO_CMD_ACK;
    packet.header.seq = seq_num;
    packet.header.payload_len = 0;
    packet.payload = nullptr;
    packet.crc = 0;
    return packet;
}
