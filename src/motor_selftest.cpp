#ifdef MOTOR_SELF_TEST
#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Display/debug_display.h"

static Stepper *leftMotor = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robot      = nullptr;

enum SelfTestPhase {
    PHASE_EN_LOW_RUN = 0,
    PHASE_EN_HIGH_RUN,
    PHASE_DONE
};

static SelfTestPhase phase = PHASE_EN_LOW_RUN;
static uint32_t phaseStartMs = 0;
static uint32_t lastLogMs = 0;

static const float TEST_CMD = 0.35f;
static const uint32_t PHASE_TIME_MS = 5000;

// Diagnostic self-test:
// - No WiFi/UDP
// - Runs motors with EN forced LOW, then EN forced HIGH
// - Helps identify whether enable polarity is the reason for "no spin"

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("SELFTEST", "Simple forward ramp test (MOTOR_SELF_TEST)");

    // Initialize debug display and show a clear test title
    debug_display_init();
    debug_display_push("TESTE", "Teste dos Motores");

    // Use the same driver configuration as main.cpp but with 1 microstep (full-step)
    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_0, LEDC_TIMER_0,
                             Stepper::DRIVER_DRV8825, 1, false);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_1, LEDC_TIMER_1,
                             Stepper::DRIVER_DRV8825, 1, true);

    robot = new Robot(*leftMotor, *rightMotor);

    // Faster ramp to avoid spending too long near zero command.
    leftMotor->set_max_accel_norm(2.0f);
    rightMotor->set_max_accel_norm(2.0f);

    // Enable torque logic inside Stepper, then force EN level per phase below.
    robot->set_torque(true);
    robot->drive_wheels(0.0f, 0.0f);

    phaseStartMs = millis();
    lastLogMs = phaseStartMs;
    DEBUG_SERIAL("SELFTEST", "Phase 1/2: forcing EN=LOW for %lu ms", (unsigned long)PHASE_TIME_MS);
    debug_display_push("TESTE", "EN LOW RUN");
}

void loop() {
    uint32_t now = millis();

    if (phase == PHASE_EN_LOW_RUN) {
        gpio_set_level(ENABLE_PIN, 0);
        robot->drive_wheels(TEST_CMD, TEST_CMD);
        if (now - phaseStartMs >= PHASE_TIME_MS) {
            phase = PHASE_EN_HIGH_RUN;
            phaseStartMs = now;
            DEBUG_SERIAL("SELFTEST", "Phase 2/2: forcing EN=HIGH for %lu ms", (unsigned long)PHASE_TIME_MS);
            debug_display_push("TESTE", "EN HIGH RUN");
        }
    } else if (phase == PHASE_EN_HIGH_RUN) {
        gpio_set_level(ENABLE_PIN, 1);
        robot->drive_wheels(TEST_CMD, TEST_CMD);
        if (now - phaseStartMs >= PHASE_TIME_MS) {
            phase = PHASE_DONE;
            phaseStartMs = now;
            robot->drive_wheels(0.0f, 0.0f);
            DEBUG_SERIAL("SELFTEST", "Done. If motor spun only in EN=LOW phase -> active-low EN. Only in EN=HIGH -> active-high EN.");
            debug_display_push("TESTE", "DONE");
        }
    } else {
        robot->drive_wheels(0.0f, 0.0f);
    }

    if (now - lastLogMs >= 500) {
        lastLogMs = now;
        DEBUG_SERIAL("SELFTEST", "phase=%d en=%d rpmL=%.1f rpmR=%.1f",
                     (int)phase,
                     (int)gpio_get_level(ENABLE_PIN),
                     (double)leftMotor->get_velocity(),
                     (double)rightMotor->get_velocity());
    }

    // Periodically update step frequencies
    leftMotor->update(0.010f);
    rightMotor->update(0.010f);
    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // MOTOR_SELF_TEST
