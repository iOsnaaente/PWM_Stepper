#ifndef GY521_IMU_H
#define GY521_IMU_H

#include <stdint.h>

// Simple MPU-6050 (GY-521) DMP-based IMU helper
// - Provides filtered, zeroed heading (deg)
// - Also computes angular velocity (deg/s) and acceleration (deg/s^2)
// - Handles an initial stabilization window and heading zeroing

class Gy521Imu {
public:
    Gy521Imu();

    // Initialize MPU-6050 and DMP. Returns true on success.
    bool begin();

    // True when DMP is initialized and ready.
    bool isReady() const;

    // Call frequently from loop(). Returns true if a new sample was processed.
    bool update();

    // Calibration / stabilization state.
    bool isCalibrating() const;
    uint32_t calibrationRemainingMs() const;

    // Filtered, zeroed heading in degrees [-180, 180].
    float getHeadingDeg() const;

    // Angular velocity (deg/s) around Z (yaw), derived from heading.
    float getAngularVelocityDps() const;

    // Angular acceleration (deg/s^2) around Z.
    float getAngularAccelerationDps2() const;

private:
    bool     _dmpReady;
    uint16_t _packetSize;

    // DMP state
    uint8_t  _fifoBuffer[64];

    // Heading filter state
    float _headingFilteredDeg;
    bool  _headingInitialized;

    float _headingOffsetDeg; // zero offset set after stabilization

    // Stabilization window
    bool     _stabilizing;
    uint32_t _stabilizeStartMs;

    // Derived kinematics
    float    _headingZeroedDeg;
    float    _omegaDps;
    float    _alphaDps2;

    bool     _havePrevHeading;
    float    _prevHeadingDeg;
    uint32_t _prevHeadingMs;

    bool     _havePrevOmega;
    float    _prevOmegaDps;
};

#endif // GY521_IMU_H
