#ifdef KINEMATICS_SELF_TEST

#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Robot/robot_kinematics.h"
#include "Display/debug_display.h"
#include "IMU/gy521_imu.h"

#include <Wire.h>
#include <math.h>

// Kinematics self-test:
// - Uses only kinematics (wheel RPM + geometry) to command motion
// - Robot performs a nominal 90-degree spin every 20 seconds
// - IMU module is used ONLY for observation (heading displayed/printed),
//   never for control or feedback.

static Stepper *leftMotor  = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robot      = nullptr;
static Gy521Imu imu;

// Spin profile configuration
static constexpr float   SPIN_RPM          = 10.0f;      // per-wheel magnitude RPM during spin
static constexpr uint32_t CYCLE_PERIOD_MS  = 20000;      // full cycle length
// Acceleration ramp (normalized units per second) to avoid skids
static constexpr float   KINEM_ACCEL_NORM_PER_S = 0.1f;   // smaller = smoother, slower ramp

// Derived from geometry at setup
static float    g_spinAngularRadPerSec = 0.0f;           // expected yaw rate from SPIN_RPM
static uint32_t g_spinDurationMs       = 0;              // duration of spin phase to get ~90 deg
static uint32_t g_idleDurationMs       = 0;              // remaining time in cycle

// Simple state for running after IMU calibration
enum TestState { IMU_CALIBRATION, WAIT_BEFORE_FIRST_TURN, TEST_RUNNING };
static TestState g_state = IMU_CALIBRATION;

// Delay between end of IMU calibration and first spin
static constexpr uint32_t FIRST_TURN_DELAY_MS = 5000; // 5 seconds
static uint32_t g_postCalibStartMs = 0;

static uint32_t g_cycleStartMs   = 0;
static uint32_t g_lastUpdateMs   = 0;
static float    g_thetaKinRad    = 0.0f;  // kinematics-based heading integration
static uint32_t g_cycleCount     = 0;
static float    g_spinNormCmd    = 0.0f;  // ramped normalized command magnitude for spin

// Map a desired wheel RPM into the stepper's normalized velocity command [-1, 1]
static float rpm_to_norm(float rpm) {
    float sign = (rpm >= 0.0f) ? 1.0f : -1.0f;
    float rpm_abs = fabsf(rpm);

    if (rpm_abs <= VEL_RPM_MIN) {
        return 0.0f; // below minimum usable RPM -> no motion
    }

    float norm_mag = (rpm_abs - VEL_RPM_MIN) / (VEL_RPM_MAX - VEL_RPM_MIN);
    if (norm_mag > 1.0f) norm_mag = 1.0f;
    return sign * norm_mag;
}

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("KINEM", "Kinematics self-test (90 deg every 20 s)");

    debug_display_init();
    debug_display_push("KINEM", "Kinematics self-test");

    // Fast I2C for both OLED and IMU
    Wire.setClock(400000);

    // Initialize IMU (will perform its own 20 s calibration window)
    if (!imu.begin()) {
        DEBUG_SERIAL("KINEM", "IMU init failed");
        debug_display_push("KINEM", "IMU INIT FAIL");
    } else {
        DEBUG_SERIAL("KINEM", "IMU module ready");
    }

    // Motors and robot as in main.cpp: DRV8825, full-step, separate timers
    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_0, LEDC_TIMER_0,
                             Stepper::DRIVER_DRV8825, 1, false);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_1, LEDC_TIMER_1,
                             Stepper::DRIVER_DRV8825, 1, true);
    robot = new Robot(*leftMotor, *rightMotor);

    robot->set_torque(false);
    robot->drive_wheels(0.0f, 0.0f);

    // Pre-compute kinematic spin profile: SPIN_RPM for each wheel, opposite directions
    float v_mps = 0.0f;
    float w_rad_s = 0.0f;
    RobotKinematics::wheel_rpm_to_body(-SPIN_RPM, +SPIN_RPM, v_mps, w_rad_s);
    g_spinAngularRadPerSec = w_rad_s;

    if (fabsf(w_rad_s) < 1e-6f) {
        g_spinDurationMs = CYCLE_PERIOD_MS; // degenerate, avoid divide-by-zero
        g_idleDurationMs = 0;
    } else {
        float t_spin_s = ( (float)M_PI / 2.0f ) / fabsf(w_rad_s); // time to rotate ~90 deg
        g_spinDurationMs = (uint32_t)(t_spin_s * 1000.0f + 0.5f);
        if (g_spinDurationMs >= CYCLE_PERIOD_MS) {
            g_spinDurationMs = CYCLE_PERIOD_MS;
            g_idleDurationMs = 0;
        } else {
            g_idleDurationMs = CYCLE_PERIOD_MS - g_spinDurationMs;
        }
    }

    DEBUG_SERIAL("KINEM", "Spin profile: SPIN_RPM=%.2f, w=%.4f rad/s, spin_ms=%lu, idle_ms=%lu",
                 (double)SPIN_RPM,
                 (double)g_spinAngularRadPerSec,
                 (unsigned long)g_spinDurationMs,
                 (unsigned long)g_idleDurationMs);

    g_cycleStartMs = millis();
    g_lastUpdateMs = g_cycleStartMs;
    g_thetaKinRad  = 0.0f;
    g_cycleCount   = 0;
    g_spinNormCmd  = 0.0f;

    g_state = IMU_CALIBRATION;
}

void loop() {
    static uint32_t lastOledMs = 0;

    uint32_t nowMs = millis();
    uint32_t dtMs  = nowMs - g_lastUpdateMs;
    if (dtMs == 0) dtMs = 1;
    float dt_sec = (float)dtMs / 1000.0f;
    g_lastUpdateMs = nowMs;

    // Poll IMU every loop. It manages its own internal FIFO and calibration.
    bool imuUpdated = imu.isReady() ? imu.update() : false;

    // While IMU is calibrating, show countdown and keep robot still.
    if (imu.isReady() && imu.isCalibrating()) {
        if (nowMs - lastOledMs >= 200) {
            lastOledMs = nowMs;
            uint32_t remainingMs = imu.calibrationRemainingMs();
            uint32_t remainingSec = (remainingMs + 999) / 1000; // round up

            char msg[64];
            snprintf(msg, sizeof(msg), "CAL %lus", (unsigned long)remainingSec);
            debug_display_push("KINEM", msg);
        }

        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);

        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    if (g_state == IMU_CALIBRATION && (!imu.isReady() || !imu.isCalibrating())) {
        // IMU calibration finished (or failed): start a post-calibration wait
        g_state = WAIT_BEFORE_FIRST_TURN;
        g_postCalibStartMs = nowMs;
        g_thetaKinRad  = 0.0f;
        g_cycleCount   = 0;
        g_spinNormCmd  = 0.0f;
        DEBUG_SERIAL("KINEM", "IMU calibration done, waiting %lu ms before first turn",
                     (unsigned long)FIRST_TURN_DELAY_MS);
    }

    if (g_state == WAIT_BEFORE_FIRST_TURN) {
        // Keep robot stopped while waiting for the first turn
        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);

        if (nowMs - g_postCalibStartMs >= FIRST_TURN_DELAY_MS) {
            g_state = TEST_RUNNING;
            g_cycleStartMs = nowMs;
            DEBUG_SERIAL("KINEM", "Starting cycles after post-calibration delay");
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
            return;
        }
    }

    if (g_state != TEST_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // Determine phase within 20 s cycle
    uint32_t cycleElapsedMs = nowMs - g_cycleStartMs;
    if (cycleElapsedMs >= CYCLE_PERIOD_MS) {
        g_cycleStartMs = nowMs;
        cycleElapsedMs = 0;
        g_thetaKinRad  = 0.0f; // reset kinematic integration each cycle
        g_cycleCount++;
        g_spinNormCmd  = 0.0f;
        DEBUG_SERIAL("KINEM", "Starting new cycle #%lu", (unsigned long)g_cycleCount);
    }

    bool inSpinPhase = (cycleElapsedMs < g_spinDurationMs);

    if (inSpinPhase) {
        // Apply spin command with acceleration ramp to avoid skids
        float targetNorm = rpm_to_norm(SPIN_RPM);
        float maxStep = KINEM_ACCEL_NORM_PER_S * dt_sec;

        if (g_spinNormCmd < targetNorm) {
            g_spinNormCmd += maxStep;
            if (g_spinNormCmd > targetNorm) g_spinNormCmd = targetNorm;
        } else if (g_spinNormCmd > targetNorm) {
            g_spinNormCmd -= maxStep;
            if (g_spinNormCmd < targetNorm) g_spinNormCmd = targetNorm;
        }

        robot->set_torque(true);
        robot->drive_wheels(-g_spinNormCmd, +g_spinNormCmd);

    // Integrate expected heading from kinematics (only during spin),
    // scaled by the current ramped command so ramps are accounted for.
    float scale = (targetNorm > 0.0f) ? (g_spinNormCmd / targetNorm) : 0.0f;
    float w_eff = g_spinAngularRadPerSec * scale;
    g_thetaKinRad += w_eff * dt_sec;

    } else {
        // Idle phase: ramp back towards zero then hold motors off
        float maxStep = KINEM_ACCEL_NORM_PER_S * dt_sec;
        if (g_spinNormCmd > 0.0f) {
            g_spinNormCmd -= maxStep;
            if (g_spinNormCmd < 0.0f) g_spinNormCmd = 0.0f;
        }

        if (g_spinNormCmd <= 0.0f) {
            robot->drive_wheels(0.0f, 0.0f);
            robot->set_torque(false);
        } else {
            robot->set_torque(true);
            robot->drive_wheels(-g_spinNormCmd, +g_spinNormCmd);
        }
    }

    // Update motor step frequencies at ~100 Hz equivalent
    leftMotor->update(dt_sec);
    rightMotor->update(dt_sec);

    // Read IMU heading (for observation only)
    float heading_deg = 0.0f;
    if (imuUpdated) {
        heading_deg = imu.getHeadingDeg();
    }

    // Kinematic expected heading in degrees for comparison
    float heading_kin_deg = g_thetaKinRad * (180.0f / (float)M_PI);

    // Serial: report both IMU and kinematics headings
    USB_BUS.printf("Cycle=%lu phase=%s H_imu=%.1f deg H_kin=%.1f deg\r\n",
                   (unsigned long)g_cycleCount,
                   inSpinPhase ? "SPIN" : "IDLE",
                   heading_deg,
                   heading_kin_deg);

    // OLED: show IMU heading only (as requested)
    if (nowMs - lastOledMs >= 100) {
        lastOledMs = nowMs;
        char msg[64];
        snprintf(msg, sizeof(msg), "H%.0f", heading_deg);
        debug_display_push("KINEM", msg);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // KINEMATICS_SELF_TEST
