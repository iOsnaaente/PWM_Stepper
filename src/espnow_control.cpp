#ifdef USE_ESPNOW_CONTROL

#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Display/debug_display.h"

#include <WiFi.h>
#include <esp_now.h>

// Simple ESP-NOW control:
// - Receives StickPacket broadcast from an external hub (e.g., WASD->ESP-NOW sender)
// - Applies same deadzone/torque/timeout logic as the WiFi/UDP mode

static Stepper *leftMotor  = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robotNow   = nullptr;

struct __attribute__((packed)) StickPacket {
    int8_t left;
    int8_t right;
    uint8_t checksum;
};

static const uint32_t COMMAND_TIMEOUT_MS = 500;   // auto-stop timeout
static TickType_t     last_command_tick  = 0;     // last time a valid command was applied
static const float    CMD_DEADZONE       = 0.05f; // treat small magnitudes as zero
static const uint32_t ESPNOW_START_DELAY_MS = 3000; // delay before bringing up WiFi/ESP-NOW

// Simple ramped commands (like motor_selftest):
// ESP-NOW callback updates targets; motor task ramps currents toward targets.
static float target_left  = 0.0f;
static float target_right = 0.0f;
static float current_left = 0.0f;
static float current_right= 0.0f;
static const float RAMP_STEP = 0.05f; // per 10 ms -> ~1.0 in ~0.2 s

static float clamp_unit(float v) { return fminf(fmaxf(v, -1.0f), 1.0f); }

// Shared watchdog: stop motors and disable torque if no command within timeout
static void control_watchdog_step() {
    if (last_command_tick != 0) {
        TickType_t now = xTaskGetTickCount();
        if ( (now - last_command_tick) * portTICK_PERIOD_MS > COMMAND_TIMEOUT_MS ) {
            robotNow->drive_wheels(0.0f, 0.0f);
            robotNow->set_torque(false);
            last_command_tick = 0;
            DEBUG_SERIAL("CTRL", "Timeout stop");
        }
    }
}

// ESP-NOW receive callback: decode StickPacket and apply control
static void onEspNowRecv(const uint8_t* mac_addr, const uint8_t* data, int len) {
    (void)mac_addr;
    if (len != (int)sizeof(StickPacket)) return;

    StickPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    uint8_t expected = (uint8_t)pkt.left ^ (uint8_t)pkt.right;
    if (expected != pkt.checksum) return;

    float left  = clamp_unit((float)pkt.left  / 127.0f);
    float right = clamp_unit((float)pkt.right / 127.0f);

    float lcmd = (fabsf(left)  < CMD_DEADZONE) ? 0.0f : left;
    float rcmd = (fabsf(right) < CMD_DEADZONE) ? 0.0f : right;

    // Update ramp targets; actual ramping happens in motor_update_Task
    target_left  = lcmd;
    target_right = rcmd;

    last_command_tick = xTaskGetTickCount();
}

static void motor_update_Task(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    for(;;) {
        // Ramp current commands towards targets (like motor_selftest)
        if (current_left < target_left) {
            current_left += RAMP_STEP;
            if (current_left > target_left) current_left = target_left;
        } else if (current_left > target_left) {
            current_left -= RAMP_STEP;
            if (current_left < target_left) current_left = target_left;
        }

        if (current_right < target_right) {
            current_right += RAMP_STEP;
            if (current_right > target_right) current_right = target_right;
        } else if (current_right > target_right) {
            current_right -= RAMP_STEP;
            if (current_right < target_right) current_right = target_right;
        }

        // Apply ramped commands to the robot
        if (!robotNow->get_torque() && (current_left != 0.0f || current_right != 0.0f)) {
            robotNow->set_torque(true);
        }
        robotNow->drive_wheels(current_left, current_right);

        leftMotor->update(0.010f);
        rightMotor->update(0.010f);
        control_watchdog_step();
        vTaskDelayUntil(&last, period);
    }
}

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("RESET", "Reset reason: %d", (int)esp_reset_reason());
    DEBUG_SERIAL("SERIAL", "ESP-NOW control mode");

    debug_display_init();
    debug_display_push("CTRL", "ESP-NOW Control");

    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_0, LEDC_TIMER_0,
                             Stepper::DRIVER_DRV8825, 1, false);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_1, LEDC_TIMER_1,
                             Stepper::DRIVER_DRV8825, 1, true);

    robotNow = new Robot(*leftMotor, *rightMotor);
    robotNow->stop();

    DEBUG_SERIAL("POWER", "Aguardando %lu ms antes de iniciar ESP-NOW/WiFi", (unsigned long)ESPNOW_START_DELAY_MS);
    delay(ESPNOW_START_DELAY_MS);

    WiFi.mode(WIFI_STA);
    DEBUG_SERIAL("ESP-NOW", "WiFi STA MAC: %s", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        DEBUG_SERIAL("ESP-NOW", "Error initializing ESP-NOW");
    } else {
        esp_now_register_recv_cb(onEspNowRecv);
        DEBUG_SERIAL("ESP-NOW", "ESP-NOW RX ready (broadcast)");
    }

    xTaskCreate(motor_update_Task, "MotorUpdate", 4096, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL);
}

#endif // USE_ESPNOW_CONTROL
