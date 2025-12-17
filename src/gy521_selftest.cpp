#ifdef GY521_SELF_TEST

#include "../board_config.h"
#include "Serial/SerialDebugger.h"
#include "Display/debug_display.h"

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// GY-521 (MPU-6050) self-test using the on-chip DMP:
// - No motors, WiFi, or ESP-NOW
// - Uses I2Cdevlib's MotionApps 2.0 DMP firmware
// - Reads yaw (heading) directly from DMP and displays it

static MPU6050 mpu;

static bool dmpReady = false;
static uint16_t packetSize = 0;
static uint8_t fifoBuffer[64];

static Quaternion q;
static VectorFloat gravity;
static float ypr[3]; // yaw, pitch, roll (radians)

// Simple low-pass filter (exponential moving average) for heading to reduce jitter
static float heading_f = 0.0f;   // filtered heading [deg]
static bool  heading_initialized = false;
static constexpr float HEADING_ALPHA = 0.8f; // 0..1, smaller = smoother

// Offset so that heading is zeroed after stabilization
static float heading_offset = 0.0f; // deg

// Stabilization window after DMP + filter startup (ms)
static bool     stabilizing = false;
static uint32_t stabilize_start_ms = 0;
static constexpr uint32_t STABILIZE_DURATION_MS = 20000; // 20 seconds

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("GY521", "GY-521 (MPU-6050 DMP) self-test mode");

    debug_display_init();
    debug_display_push("GY521", "MPU-6050 DMP test");

    // Fast I2C for both OLED and MPU
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        DEBUG_SERIAL("GY521", "MPU-6050 connection failed");
        return;
    }

    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        DEBUG_SERIAL("GY521", "DMP ready (packetSize=%u)", packetSize);

        // Start stabilization timer: allow DMP + filter to settle for 10 s
        stabilizing = true;
        stabilize_start_ms = millis();
        heading_initialized = false;
    } else {
        DEBUG_SERIAL("GY521", "DMP init failed (code %u)", devStatus);
    }
}

void loop() {
    static uint32_t lastOledMs = 0;

    if (!dmpReady) {
        delay(10);
        return;
    }

    // Get the latest DMP packet if available
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // No new packet this loop
        return;
    }

    // Orientation from DMP
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw_deg = ypr[0] * (180.0f / 3.14159265f);

    // Wrap-aware exponential moving average for heading
    if (!heading_initialized) {
        heading_f = yaw_deg;
        heading_initialized = true;
    } else {
        float diff = yaw_deg - heading_f;
        // Wrap into [-180,180] to avoid jumps across 0/360
        while (diff > 180.0f)  diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        // If rotation since last sample is large, trust DMP directly to
        // avoid the filter "losing" heading during quick spins.
        if (fabsf(diff) > 30.0f) {
            heading_f = yaw_deg;
        } else {
            heading_f += HEADING_ALPHA * diff;
        }
    }

    uint32_t nowMs = millis();
    // During initial stabilization window, run filter but show countdown
    if (stabilizing && (nowMs - stabilize_start_ms < STABILIZE_DURATION_MS)) {
        if (nowMs - lastOledMs >= 200) {
            lastOledMs = nowMs;
            uint32_t elapsed = nowMs - stabilize_start_ms;
            uint32_t remainingMs = (STABILIZE_DURATION_MS > elapsed) ? (STABILIZE_DURATION_MS - elapsed) : 0;
            uint32_t remainingSec = (remainingMs + 999) / 1000; // round up

            char msg[64];
            snprintf(msg, sizeof(msg), "CAL %lus", static_cast<unsigned long>(remainingSec));
            debug_display_push("GY521", msg);
        }
        return;
    } else if (stabilizing) {
        // End stabilization period and zero heading at this point
        stabilizing = false;
        heading_offset = heading_f;
    }

    // Apply zero offset and wrap to [-180,180]
    float heading_zero = heading_f - heading_offset;
    while (heading_zero > 180.0f)  heading_zero -= 360.0f;
    while (heading_zero < -180.0f) heading_zero += 360.0f;

    // Serial output: filtered, zeroed heading after stabilization
    USB_BUS.printf("Heading[deg]=%.1f\r\n", heading_zero);

    // Throttled OLED output for readability
    if (nowMs - lastOledMs >= 100) {
        lastOledMs = nowMs;
        char msg[64];
        snprintf(msg, sizeof(msg), "H%.0f", heading_zero);
        debug_display_push("GY521", msg);
    }
}

#endif // GY521_SELF_TEST
