#ifdef GY521_SELF_TEST

#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Display/debug_display.h"
#include "IMU/gy521_imu.h"

#include <Wire.h>

// GY-521 (MPU-6050) self-test using the shared Gy521Imu module:
// - No motors, WiFi, or ESP-NOW
// - Uses I2Cdevlib's MotionApps 2.0 DMP firmware under the hood
// - Exposes heading, angular velocity, and angular acceleration

static Gy521Imu imu;

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("GY521", "GY-521 (MPU-6050 DMP) self-test mode");

    debug_display_init();
    debug_display_push("GY521", "MPU-6050 DMP test");

    // Fast I2C for both OLED and MPU
    Wire.setClock(400000);

    if (!imu.begin()) {
        DEBUG_SERIAL("GY521", "MPU-6050 DMP init failed");
        debug_display_push("GY521", "IMU INIT FAIL");
        return;
    }

    DEBUG_SERIAL("GY521", "GY521 IMU module ready");
}

void loop() {
    static uint32_t lastOledMs = 0;

    if (!imu.isReady()) {
        delay(10);
        return;
    }

    uint32_t nowMs = millis();
    bool updated = imu.update();

    // During initial stabilization window, show countdown
    if (imu.isCalibrating()) {
        if (nowMs - lastOledMs >= 200) {
            lastOledMs = nowMs;
            uint32_t remainingMs = imu.calibrationRemainingMs();
            uint32_t remainingSec = (remainingMs + 999) / 1000; // round up

            char msg[64];
            snprintf(msg, sizeof(msg), "CAL %lus", static_cast<unsigned long>(remainingSec));
            debug_display_push("GY521", msg);
        }
        return;
    }

    if (!updated) {
        // No new sample to show
        return;
    }

    float heading_deg = imu.getHeadingDeg();
    float omega_dps = imu.getAngularVelocityDps();
    float alpha_dps2 = imu.getAngularAccelerationDps2();

    // Serial output: filtered, zeroed heading after stabilization
    USB_BUS.printf("H=%.1f deg, W=%.1f deg/s, A=%.1f deg/s2\r\n",
                   heading_deg, omega_dps, alpha_dps2);

    // Throttled OLED output for readability
    if (nowMs - lastOledMs >= 100) {
        lastOledMs = nowMs;
        char msg[64];
        snprintf(msg, sizeof(msg), "H%.0f", heading_deg);
        debug_display_push("GY521", msg);
    }
}

#endif // GY521_SELF_TEST
