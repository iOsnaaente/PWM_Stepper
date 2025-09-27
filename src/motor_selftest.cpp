#ifdef MOTOR_SELF_TEST
#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"

// Optional: enable direct freq + verbose if not already in build flags
//#define STEPPER_DIRECT_FREQ_MODE
//#define DEBUG_MOTOR_VERBOSE

static Stepper *leftMotor = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robot      = nullptr;

static void ramp_wheel(Stepper* m, const char* name, bool bidir) {
    // Ramp up
    for (int i=0;i<=10;i++) {
        float norm = i/10.0f; if (norm<0.05f) norm=0.05f; // avoid ultra-low freq
        m->set_velocity(norm);
        DEBUG_SERIAL("TEST", "%s +%.2f", name, norm);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (bidir) {
        // Ramp down negative
        for (int i=10;i>=0;i--) {
            float norm = -i/10.0f; if (norm>-0.05f) norm=-0.05f;
            m->set_velocity(norm);
            DEBUG_SERIAL("TEST", "%s %.2f", name, norm);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        m->set_velocity(0.0f);
    }
}

static void drive_pattern() {
    DEBUG_SERIAL("TEST", "Robot::drive pattern (vel,turn)");
    // Forward accelerate
    for (int i=0;i<=10;i++) {
        float v = i/10.0f; if (v<0.1f) v=0.1f;
        robot->drive(v, 0.0f);
        DEBUG_SERIAL("DRIVE", "vel=%.2f turn=0.00", v);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    // Gentle right arc (positive turn reduces left wheel or increases right depending on formula)
    for (int i=0;i<=10;i++) {
        float t = i/20.0f; // 0 .. 0.5
        robot->drive(0.6f, t);
        DEBUG_SERIAL("DRIVE", "vel=0.60 turn=%.2f", t);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    // Pivot in place left, then right
    robot->drive(0.0f, -0.8f); DEBUG_SERIAL("DRIVE", "pivot left turn=-0.80"); vTaskDelay(pdMS_TO_TICKS(1500));
    robot->drive(0.0f,  0.8f); DEBUG_SERIAL("DRIVE", "pivot right turn=0.80"); vTaskDelay(pdMS_TO_TICKS(1500));
    // Reverse slow
    for (int i=0;i<=8;i++) {
        float v = -i/10.0f; // 0 .. -0.8
        robot->drive(v, 0.0f);
        DEBUG_SERIAL("DRIVE", "vel=%.2f turn=0.00", v);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    robot->stop();
}

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("SELFTEST", "Motor self-test mode ativo");

    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN, LEDC_CHANNEL_1, LEDC_TIMER_0);
    robot = new Robot(*leftMotor, *rightMotor);
    robot->stop();

#ifdef STEPPER_DIRECT_FREQ_MODE
    DEBUG_SERIAL("SELFTEST", "Modo STEPPER_DIRECT_FREQ_MODE");
#else
    DEBUG_SERIAL("SELFTEST", "Modo RPM normal (defina STEPPER_DIRECT_FREQ_MODE para freq direta)");
#endif

    DEBUG_SERIAL("SELFTEST", "Sequencia: Left fwd ramp, Left rev ramp, Right fwd ramp, Right rev ramp, Both spin opposite");
}

void loop() {
    // Left motor test
    ramp_wheel(leftMotor, "LEFT", true);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Right motor test
    ramp_wheel(rightMotor, "RIGHT", true);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Opposite spin test
    DEBUG_SERIAL("TEST", "Opposite spin test");
    leftMotor->set_velocity( 0.8f);
    rightMotor->set_velocity(-0.8f);
    vTaskDelay(pdMS_TO_TICKS(3000));
    robot->stop();

    // Combined Robot::drive tests
    drive_pattern();

    DEBUG_SERIAL("SELFTEST", "Ciclo completo. Reiniciando em 5s");
    vTaskDelay(pdMS_TO_TICKS(5000));
}

#endif // MOTOR_SELF_TEST
