/**
 * @file    proto_base.h
 * @brief   Definicoes basicas do protocolo de comunicacao (pack/unpack + CRC).
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Criado em 24 de novembro de 2025
 */

#pragma once


#include "proto_base_utils.h"

class ProtoComm {
  public:

    static size_t pack(
      ProtoPacket_t &packet_in,
      uint8_t *buffer_out,
      size_t buffer_out_size
    ){
      const size_t total_len =
        PROTO_BASE_LEN + packet_in.header.payload_len;
      if (buffer_out_size < total_len) {
        return 0;
      }

      size_t idx = 0;
      buffer_out[idx++] = packet_in.header.flags;
      buffer_out[idx++] = packet_in.header.id;
      buffer_out[idx++] = packet_in.header.cmd;
      buffer_out[idx++] = (uint8_t)(packet_in.header.seq & 0xFF);
      buffer_out[idx++] = (uint8_t)((packet_in.header.seq >> 8) & 0xFF);
      buffer_out[idx++] = packet_in.header.payload_len;

      if ( packet_in.header.payload_len && packet_in.payload ) {
        std::memcpy(
          buffer_out + PROTO_HEADER_LEN,
          packet_in.payload,
          packet_in.header.payload_len
        );
        idx += packet_in.header.payload_len;
      }

      uint16_t crc = ProtoComm::crc16(
        buffer_out,
        PROTO_HEADER_LEN + packet_in.header.payload_len
      );
      buffer_out[idx++] = (uint8_t)(crc & 0xFF);
      buffer_out[idx++] = (uint8_t)((crc >> 8) & 0xFF);

      return total_len;
    }

    static bool unpack(
      const uint8_t *buffer_in,
      size_t buffer_in_size,
      ProtoPacket_t &packet_out
    ){
      if (buffer_in_size < PROTO_BASE_LEN) {
        return false;
      }

      size_t idx = 0;
      packet_out.header.flags = buffer_in[idx++];
      packet_out.header.id = buffer_in[idx++];
      packet_out.header.cmd = buffer_in[idx++];
      packet_out.header.seq = (uint16_t)buffer_in[idx++];
      packet_out.header.seq |= (uint16_t)buffer_in[idx++] << 8;
      packet_out.header.payload_len = buffer_in[idx++];

      if (buffer_in_size <
          (size_t)PROTO_BASE_LEN + packet_out.header.payload_len) {
        return false;
      }

      if ( packet_out.header.payload_len > 0 ) {
        packet_out.payload = new uint8_t[packet_out.header.payload_len];
        std::memcpy(
          packet_out.payload,
          buffer_in + PROTO_HEADER_LEN,
          packet_out.header.payload_len
        );
      } else {
        packet_out.payload = nullptr;
      }
      idx += packet_out.header.payload_len;

      if (idx + 1 >= buffer_in_size) {
        ProtoComm::cleanup( packet_out );
        return false;
      }

      packet_out.crc = (uint16_t)buffer_in[idx++];
      packet_out.crc |= (uint16_t)buffer_in[idx++] << 8;

      uint16_t computed_crc = ProtoComm::crc16(
        buffer_in,
        PROTO_HEADER_LEN + packet_out.header.payload_len
      );

      if (computed_crc != packet_out.crc) {
        ProtoComm::cleanup( packet_out );
        return false;
      }

      return true;
    }

    static bool cleanup( ProtoPacket_t &packet ){
      if (packet.payload) {
        delete[] packet.payload;
        packet.payload = nullptr;
        return true;
      }
      return false;
    }

    static uint16_t crc16(
      const uint8_t *data, size_t len
    ){
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
          if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
          } else {
            crc <<= 1;
          }
        }
      }
      return crc;
    }
};
