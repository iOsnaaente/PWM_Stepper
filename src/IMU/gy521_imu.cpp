#include "gy521_imu.h"

#include <math.h>
#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Local MPU6050 instance for this module
static MPU6050 s_mpu;

// DMP helper objects
static Quaternion   s_q;
static VectorFloat  s_gravity;
static float        s_ypr[3]; // yaw, pitch, roll (radians)

// Filter and timing constants
static constexpr float   HEADING_ALPHA             = 0.8f;     // 0..1, smaller = smoother
static constexpr float   LARGE_ROTATION_THRESHOLD  = 30.0f;    // deg per sample to snap filter
static constexpr uint32_t STABILIZE_DURATION_MS    = 20000;    // 20 seconds

Gy521Imu::Gy521Imu()
    : _dmpReady(false),
      _packetSize(0),
      _headingFilteredDeg(0.0f),
      _headingInitialized(false),
      _headingOffsetDeg(0.0f),
      _stabilizing(false),
      _stabilizeStartMs(0),
      _headingZeroedDeg(0.0f),
      _omegaDps(0.0f),
      _alphaDps2(0.0f),
      _havePrevHeading(false),
      _prevHeadingDeg(0.0f),
      _prevHeadingMs(0),
      _havePrevOmega(false),
      _prevOmegaDps(0.0f) {
}

bool Gy521Imu::begin() {
    // Assume Wire clock is configured outside if needed.
    s_mpu.initialize();
    if (!s_mpu.testConnection()) {
        _dmpReady = false;
        return false;
    }

    uint8_t devStatus = s_mpu.dmpInitialize();
    if (devStatus != 0) {
        _dmpReady = false;
        return false;
    }

    s_mpu.setDMPEnabled(true);
    _packetSize = s_mpu.dmpGetFIFOPacketSize();
    _dmpReady = true;

    _stabilizing = true;
    _stabilizeStartMs = millis();
    _headingInitialized = false;
    _havePrevHeading = false;
    _havePrevOmega = false;
    _headingOffsetDeg = 0.0f;
    _headingZeroedDeg = 0.0f;
    _omegaDps = 0.0f;
    _alphaDps2 = 0.0f;

    return true;
}

bool Gy521Imu::isReady() const {
    return _dmpReady;
}

bool Gy521Imu::isCalibrating() const {
    if (!_stabilizing) {
        return false;
    }
    uint32_t nowMs = millis();
    return (nowMs - _stabilizeStartMs) < STABILIZE_DURATION_MS;
}

uint32_t Gy521Imu::calibrationRemainingMs() const {
    if (!_stabilizing) {
        return 0;
    }
    uint32_t nowMs = millis();
    uint32_t elapsed = nowMs - _stabilizeStartMs;
    if (elapsed >= STABILIZE_DURATION_MS) {
        return 0;
    }
    return STABILIZE_DURATION_MS - elapsed;
}

bool Gy521Imu::update() {
    if (!_dmpReady) {
        return false;
    }

    // Get latest DMP packet if available
    if (!s_mpu.dmpGetCurrentFIFOPacket(_fifoBuffer)) {
        return false; // no new data
    }

    // Orientation from DMP
    s_mpu.dmpGetQuaternion(&s_q, _fifoBuffer);
    s_mpu.dmpGetGravity(&s_gravity, &s_q);
    s_mpu.dmpGetYawPitchRoll(s_ypr, &s_q, &s_gravity);

    float yawDeg = s_ypr[0] * (180.0f / 3.14159265f);

    // Wrap-aware exponential moving average for heading
    if (!_headingInitialized) {
        _headingFilteredDeg = yawDeg;
        _headingInitialized = true;
    } else {
        float diff = yawDeg - _headingFilteredDeg;
        // Wrap into [-180, 180] to avoid jumps across 0/360
        while (diff > 180.0f)  diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        if (fabsf(diff) > LARGE_ROTATION_THRESHOLD) {
            // Large instantaneous rotation: trust DMP directly
            _headingFilteredDeg = yawDeg;
        } else {
            _headingFilteredDeg += HEADING_ALPHA * diff;
        }
    }

    uint32_t nowMs = millis();

    // Handle stabilization / zeroing
    if (_stabilizing && (nowMs - _stabilizeStartMs < STABILIZE_DURATION_MS)) {
        // Still stabilizing; do not update zeroed heading or derivatives yet
        return true; // data processed, but not yet "valid" for consumers
    } else if (_stabilizing) {
        // End stabilization period and zero heading at this point
        _stabilizing = false;
        _headingOffsetDeg = _headingFilteredDeg;

        _headingZeroedDeg = 0.0f;
        _omegaDps = 0.0f;
        _alphaDps2 = 0.0f;

        _havePrevHeading = false;
        _havePrevOmega = false;

        return true;
    }

    // Apply zero offset and wrap to [-180, 180]
    float headingZero = _headingFilteredDeg - _headingOffsetDeg;
    while (headingZero > 180.0f)  headingZero -= 360.0f;
    while (headingZero < -180.0f) headingZero += 360.0f;

    _headingZeroedDeg = headingZero;

    // Compute angular velocity and acceleration from heading
    if (!_havePrevHeading) {
        _havePrevHeading = true;
        _prevHeadingDeg = headingZero;
        _prevHeadingMs = nowMs;
        _omegaDps = 0.0f;
        _alphaDps2 = 0.0f;
        _havePrevOmega = false;
    } else {
        uint32_t dtMs = nowMs - _prevHeadingMs;
        float dt = dtMs > 0 ? (static_cast<float>(dtMs) / 1000.0f) : 1e-3f;

        float dHeading = headingZero - _prevHeadingDeg;
        while (dHeading > 180.0f)  dHeading -= 360.0f;
        while (dHeading < -180.0f) dHeading += 360.0f;

        float omega = dHeading / dt; // deg/s

        if (!_havePrevOmega) {
            _omegaDps = omega;
            _alphaDps2 = 0.0f;
            _havePrevOmega = true;
        } else {
            float dOmega = omega - _prevOmegaDps;
            float alpha = dOmega / dt; // deg/s^2
            _omegaDps = omega;
            _alphaDps2 = alpha;
        }

        _prevHeadingDeg = headingZero;
        _prevHeadingMs = nowMs;
        _prevOmegaDps = _omegaDps;
    }

    return true;
}

float Gy521Imu::getHeadingDeg() const {
    return _headingZeroedDeg;
}

float Gy521Imu::getAngularVelocityDps() const {
    return _omegaDps;
}

float Gy521Imu::getAngularAccelerationDps2() const {
    return _alphaDps2;
}

