#ifdef STRAIGHT_SELF_TEST

#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Stepper/stepper.h"
#include "Robot/robot.h"
#include "Robot/robot_kinematics.h"
#include "Display/debug_display.h"
#include "IMU/gy521_imu.h"

#include <Wire.h>
#include <math.h>

// Straight-line kinematics self-test:
// - Uses only kinematics (wheel RPM + geometry) to command motion
// - Robot drives straight forward for ~80 mm with a slow ramped profile
// - IMU module is used ONLY for observation (heading displayed/printed),
//   never for control or feedback.

static Stepper *leftMotor  = nullptr;
static Stepper *rightMotor = nullptr;
static Robot   *robot      = nullptr;
static Gy521Imu imu;

// Linear motion configuration
// Positive RPM should correspond to physical forward motion; if your wiring
// or mounting inverts this, adjust the sign here.
static constexpr float   LINE_RPM           = 6.0f;      // per-wheel RPM during straight motion
static constexpr float   TARGET_DIST_M      = 0.500f;     // 80 mm target path length
static constexpr float   LINE_ACCEL_NORM_PER_S = 0.10f;   // ramp rate in norm/s (slow and smooth)
// Empirical linear calibration.
//  - First run: commanded 80 mm, measured ~700 mm -> initial gain ~8.75.
//  - Latest run with that gain: commanded 80 mm, measured ~91.6 mm.
// Since actual distance is inversely proportional to this gain for a fixed
// kinematic threshold, we rescale the gain by (91.6/80) so the next run
// should be much closer to 80 mm. Combined gain ~= 8.75 * (91.6/80) ~= 10.0.
static constexpr float   DIST_GAIN          = 14.1f;

// Simple state machine
enum TestState { IMU_CALIBRATION, WAIT_BEFORE_MOVE, MOVING_FORWARD, DONE };
static TestState g_state = IMU_CALIBRATION;

static constexpr uint32_t FIRST_MOVE_DELAY_MS = 5000; // wait 5 s after calibration

static uint32_t g_lastUpdateMs      = 0;
static uint32_t g_postCalibStartMs  = 0;

static float    g_vNominal_mps      = 0.0f; // nominal forward speed at LINE_RPM
static float    g_sKin_m            = 0.0f; // integrated distance from kinematics
static float    g_moveNormCmd       = 0.0f; // ramped normalized command magnitude

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
    DEBUG_SERIAL("STRAIGHT", "Straight-line self-test (80 mm)");

    debug_display_init();
    debug_display_push("STRAIGHT", "Straight self-test");

    // Fast I2C for both OLED and IMU
    Wire.setClock(400000);

    // Initialize IMU (will perform its own 20 s calibration window)
    if (!imu.begin()) {
        DEBUG_SERIAL("STRAIGHT", "IMU init failed");
        debug_display_push("STRAIGHT", "IMU INIT FAIL");
    } else {
        DEBUG_SERIAL("STRAIGHT", "IMU module ready");
    }

    // Motors and robot as in main.cpp: A4988, full-step, separate timers
    leftMotor  = new Stepper(M1_VEL_PIN, M1_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_0, LEDC_TIMER_0,
                             Stepper::DRIVER_A4988, 1, false);
    rightMotor = new Stepper(M2_VEL_PIN, M2_DIR_PIN, ENABLE_PIN,
                             LEDC_CHANNEL_1, LEDC_TIMER_1,
                             Stepper::DRIVER_A4988, 1, true);
    robot = new Robot(*leftMotor, *rightMotor);

    robot->set_torque(false);
    robot->drive_wheels(0.0f, 0.0f);

    // Pre-compute nominal forward speed magnitude from kinematics at LINE_RPM
    float v_mps = 0.0f;
    float w_rad_s = 0.0f;
    RobotKinematics::wheel_rpm_to_body(LINE_RPM, LINE_RPM, v_mps, w_rad_s);
    g_vNominal_mps = fabsf(v_mps) * DIST_GAIN; // calibrated magnitude; direction via command sign

    DEBUG_SERIAL("STRAIGHT", "Nominal |v|=%.4f m/s (cal) at %.1f RPM", (double)g_vNominal_mps, (double)LINE_RPM);

    g_lastUpdateMs     = millis();
    g_postCalibStartMs = 0;
    g_sKin_m           = 0.0f;
    g_moveNormCmd      = 0.0f;
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
            debug_display_push("STRAIGHT", msg);
        }

        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);

        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // Transition from calibration to a short wait before motion
    if (g_state == IMU_CALIBRATION && (!imu.isReady() || !imu.isCalibrating())) {
        g_state = WAIT_BEFORE_MOVE;
        g_postCalibStartMs = nowMs;
        g_sKin_m      = 0.0f;
        g_moveNormCmd = 0.0f;
        DEBUG_SERIAL("STRAIGHT", "IMU calibration done, waiting %lu ms before move",
                     (unsigned long)FIRST_MOVE_DELAY_MS);
    }

    if (g_state == WAIT_BEFORE_MOVE) {
        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);

        if (nowMs - g_postCalibStartMs >= FIRST_MOVE_DELAY_MS) {
            g_state = MOVING_FORWARD;
            g_sKin_m      = 0.0f;
            g_moveNormCmd = 0.0f;
            DEBUG_SERIAL("STRAIGHT", "Starting forward motion to %.1f mm", (double)(TARGET_DIST_M * 1000.0f));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
            return;
        }
    }

    if (g_state == MOVING_FORWARD) {
        // Ramp normalized command toward target based on LINE_RPM
        float targetNorm = rpm_to_norm(LINE_RPM);
        float maxStep = LINE_ACCEL_NORM_PER_S * dt_sec;

        if (g_moveNormCmd < targetNorm) {
            g_moveNormCmd += maxStep;
            if (g_moveNormCmd > targetNorm) g_moveNormCmd = targetNorm;
        } else if (g_moveNormCmd > targetNorm) {
            g_moveNormCmd -= maxStep;
            if (g_moveNormCmd < targetNorm) g_moveNormCmd = targetNorm;
        }

        robot->set_torque(true);
        robot->drive_wheels(g_moveNormCmd, g_moveNormCmd);

        // Effective forward speed magnitude considering ramp
        float scale = (targetNorm > 0.0f) ? (g_moveNormCmd / targetNorm) : 0.0f;
        float v_eff = g_vNominal_mps * scale;
        g_sKin_m += v_eff * dt_sec;

        if (g_sKin_m >= TARGET_DIST_M) {
            g_state = DONE;
            robot->drive_wheels(0.0f, 0.0f);
            robot->set_torque(false);
            DEBUG_SERIAL("STRAIGHT", "Target distance reached (%.1f mm)", (double)(g_sKin_m * 1000.0f));
        }
    }

    if (g_state == DONE) {
        // Hold stopped, continue IMU display
        robot->drive_wheels(0.0f, 0.0f);
        robot->set_torque(false);
    }

    // Update motor step frequencies
    leftMotor->update(dt_sec);
    rightMotor->update(dt_sec);

    // Read IMU heading (for observation only)
    float heading_deg = 0.0f;
    if (imuUpdated) {
        heading_deg = imu.getHeadingDeg();
    }

    // Serial: report IMU heading and kinematic distance
    USB_BUS.printf("state=%d S_kin=%.1f mm H_imu=%.1f deg\r\n",
                   (int)g_state,
                   g_sKin_m * 1000.0f,
                   heading_deg);

    // OLED: show IMU heading and expected distance from kinematics
    if (nowMs - lastOledMs >= 100) {
        lastOledMs = nowMs;
        char msg[64];
        // First line: heading
        snprintf(msg, sizeof(msg), "H%.0f", heading_deg);
        debug_display_push("STRAIGHT", msg);

        // Second line: expected distance in mm (rounded)
        snprintf(msg, sizeof(msg), "S%.0fmm", g_sKin_m * 1000.0f);
        debug_display_push("STRAIGHT", msg);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // STRAIGHT_SELF_TEST
