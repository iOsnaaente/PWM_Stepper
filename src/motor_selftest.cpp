#ifdef MOTOR_SELF_TEST
#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Display/debug_display.h"

static Stepper *leftMotor = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robot      = nullptr;

// Simple forward-run self-test:
// - No WiFi/UDP
// - Both wheels ramp forward from 0 to 1.0 and hold

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("SELFTEST", "Simple forward ramp test (MOTOR_SELF_TEST)");

    // Initialize debug display and show a clear test title
    debug_display_init();
    debug_display_push("TESTE", "Teste dos Motores");

    // Use the same driver configuration as main.cpp but with 1 microstep (full-step)
    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_0, LEDC_TIMER_0,
                             Stepper::DRIVER_A4988, 1, false);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_1, LEDC_TIMER_1,
                             Stepper::DRIVER_A4988, 1, true);

    robot = new Robot(*leftMotor, *rightMotor);

    // Enable torque and start stopped (we'll ramp in loop)
    robot->set_torque(true);
    robot->drive_wheels(0.0f, 0.0f);
}

void loop() {
    // Simple ramp from 0.0 to 1.0 over ~2 seconds, then hold
    static float cmd = 0.0f;
    if (cmd < 1.0f) {
        cmd += 0.01f; // 0.01 per 10 ms -> ~1.0 in ~1 s; adjust as needed
        if (cmd > 1.0f) cmd = 1.0f;
        robot->drive_wheels(cmd, cmd);
    }

    // Periodically update step frequencies
    leftMotor->update(0.010f);
    rightMotor->update(0.010f);
    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // MOTOR_SELF_TEST
