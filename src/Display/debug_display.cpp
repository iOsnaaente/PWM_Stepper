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
                    draw_base();
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
