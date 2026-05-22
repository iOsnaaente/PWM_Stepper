/**
 * @file    espnow_comm_utils.h
 * @brief   Auxiliary types for ESPNowComm (broadcast, fixed channel).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default broadcast MAC. Use this as peer_mac for broadcast operation.
#define ESPNOW_BROADCAST_MAC { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

typedef struct {
    uint8_t channel;       // 1..13, fixed. Must match peer.
    uint8_t peer_mac[6];   // FF:FF:FF:FF:FF:FF for broadcast.
    bool    auto_start;    // If true, init() automatically calls start().
} espnow_comm_config_t;

#ifdef __cplusplus
}
#endif
