#ifndef DEBUG_DISPLAY_H_
#define DEBUG_DISPLAY_H_

#include "../board_config.h"
#include <U8g2lib.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C" {
#endif

void debug_display_init();
void debug_display_push(const char* type, const char* msg); // thread-safe (uses internal mutex)
// Define IP (string form) to be shown alternating with baseline every 10 seconds
void debug_display_set_ip(const char* ipStr);

#ifdef __cplusplus
}
#endif

#endif // DEBUG_DISPLAY_H_
