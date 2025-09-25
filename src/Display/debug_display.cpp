#include "Display/debug_display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

// Full buffer instance (assuming 128x32). If display is 128x64 later, adjust here.
static U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static SemaphoreHandle_t dispMutex = nullptr;
static char lastType[12] = {0};
static char lastMsg[64]  = {0};
static char baseL1[22]   = "Engenharia de";
static char baseL2[22]   = "controle e automacao";
static char ipLine[22]   = {0}; // "IP: x.x.x.x" stored here (fits 21 chars)
static bool haveIP       = false;
static bool showIPScreen = false; // toggled every interval when idle

// Interval for alternating baseline/IP when no overlay
#define TOGGLE_INTERVAL_MS 10000
static TickType_t nextToggleTick = 0;
static bool showingOverlay = false;
static TickType_t overlayDeadline = 0; // tick when overlay should clear

#define OVERLAY_DURATION_MS 5000

static void draw_base() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_luRS08_tr);
    uint8_t w = u8g2.getDisplayWidth();
    int y = 10;
    u8g2.drawUTF8((w - u8g2.getUTF8Width(baseL1))/2, y, baseL1); y += 11;
    u8g2.drawUTF8((w - u8g2.getUTF8Width(baseL2))/2, y, baseL2);
    u8g2.sendBuffer();
}

static void draw_ip_screen() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_luRS08_tr);
    uint8_t w = u8g2.getDisplayWidth();
    const char* title = "Endereco IP"; // fits within width
    int y = 10;
    u8g2.drawUTF8((w - u8g2.getUTF8Width(title))/2, y, title); y += 11;
    if (haveIP) {
        u8g2.drawUTF8((w - u8g2.getUTF8Width(ipLine))/2, y, ipLine);
    } else {
        const char* none = "(a obter...)";
        u8g2.drawUTF8((w - u8g2.getUTF8Width(none))/2, y, none);
    }
    u8g2.sendBuffer();
}

static void draw_overlay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf); // smaller for more text
    // First line: type
    char header[20];
    snprintf(header, sizeof(header), "[%s]", lastType);
    u8g2.drawUTF8(0, 8, header);
    // Wrap message manually if longer than width (~21 char with this font)
    const int maxChars = 21;
    char line1[32]={0}, line2[32]={0};
    size_t len = strlen(lastMsg);
    if (len <= (size_t)maxChars) {
        strncpy(line1, lastMsg, sizeof(line1)-1);
    } else {
        strncpy(line1, lastMsg, maxChars);
        strncpy(line2, lastMsg + maxChars, maxChars);
    }
    u8g2.drawUTF8(0, 18, line1);
    if (line2[0]) u8g2.drawUTF8(0, 28, line2);
    u8g2.sendBuffer();
}

static void display_task(void* arg) {
    for(;;) {
        if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (showingOverlay) {
                if ((int32_t)(xTaskGetTickCount() - overlayDeadline) >= 0) {
                    // time expired -> revert
                    showingOverlay = false;
                    // force a redraw of whichever idle screen is current
                    if (showIPScreen) draw_ip_screen(); else draw_base();
                }
            } else {
                // Idle (no overlay): handle periodic toggle
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(now - nextToggleTick) >= 0) {
                    showIPScreen = !showIPScreen && haveIP; // only switch to IP if we actually have it
                    nextToggleTick = now + pdMS_TO_TICKS(TOGGLE_INTERVAL_MS);
                    if (showIPScreen) draw_ip_screen(); else draw_base();
                }
            }
            xSemaphoreGive(dispMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void debug_display_init() {
    dispMutex = xSemaphoreCreateMutex();
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    u8g2.setBusClock(400000);
    u8g2.begin();
    draw_base();
    nextToggleTick = xTaskGetTickCount() + pdMS_TO_TICKS(TOGGLE_INTERVAL_MS);
    xTaskCreatePinnedToCore(display_task, "DispTask", 2048, NULL, 1, NULL, 0);
}

void debug_display_push(const char* type, const char* msg) {
    if (!dispMutex) return;
    if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        strncpy(lastType, type ? type : "?", sizeof(lastType)-1);
        strncpy(lastMsg, msg ? msg : "", sizeof(lastMsg)-1);
        lastType[sizeof(lastType)-1]=0; lastMsg[sizeof(lastMsg)-1]=0;
        showingOverlay = true;
        overlayDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(OVERLAY_DURATION_MS);
        draw_overlay();
        xSemaphoreGive(dispMutex);
    }
}

void debug_display_set_ip(const char* ipStr) {
    if (!dispMutex) return;
    if (xSemaphoreTake(dispMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (ipStr && *ipStr) {
            snprintf(ipLine, sizeof(ipLine), "IP: %s", ipStr);
            haveIP = true;
        } else {
            ipLine[0] = '\0';
            haveIP = false;
            showIPScreen = false; // revert to baseline only
        }
        // If currently showing IP screen, refresh
        if (!showingOverlay) {
            if (showIPScreen && haveIP) draw_ip_screen();
        }
        xSemaphoreGive(dispMutex);
    }
}
