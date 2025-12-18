#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>

// Differential-drive kinematics helpers for this robot.
// Uses the actual wheel and geometry values:
//  - Two wheels, 58.25 mm apart (center-to-center track width)
//  - Wheel diameter: 69.8 mm
//  - Stepper step angle: 1.8 degrees (full step)
// All units are SI (meters, seconds, radians) where applicable.

namespace RobotKinematics {

// Geometry constants
static constexpr float WHEEL_DIAMETER_M   = 0.0698f;   // 69.8 mm
static constexpr float WHEEL_RADIUS_M     = WHEEL_DIAMETER_M * 0.5f;
static constexpr float WHEEL_BASE_M       = 0.05825f;  // 58.25 mm between wheel centers

// Empirical angular calibration factor.
// Latest calibration:
//  - With ANGULAR_GAIN = 11.5 the 90-degree self-test produced ~103 degrees.
//  - Since achieved angle is inversely proportional to this gain, we scale the
//    gain by (103/90) so that the next 90-degree command should be closer to
//    target: new_gain = 11.5 * 103 / 90 ≈ 13.16.
static constexpr float ANGULAR_GAIN       = 13.16f;

// Stepper properties
static constexpr float STEP_ANGLE_DEG     = 1.8f;      // full-step angle
static constexpr float STEPS_PER_REV      = 360.0f / STEP_ANGLE_DEG; // 200 full steps per rev

// Conversion helpers
float rpm_to_rad_per_sec(float rpm);
float rpm_to_linear_m_per_s(float rpm);

// Given left/right wheel RPM, compute body-frame linear and angular velocities.
//  - v_mps: forward linear speed of robot center [m/s]
//  - w_rad_s: yaw rate about vertical axis [rad/s]
void wheel_rpm_to_body(float left_rpm, float right_rpm,
                       float& v_mps, float& w_rad_s);

// Step rate helpers (full steps per second).
float steps_per_sec_to_rpm(float steps_per_sec);

// Given left/right full-step rates [steps/s], compute body-frame velocities.
void step_rates_to_body(float left_steps_per_s, float right_steps_per_s,
                        float& v_mps, float& w_rad_s);

// Convenience: given desired body velocities, compute ideal wheel RPMs.
//  - v_mps: desired forward linear speed [m/s]
//  - w_rad_s: desired yaw rate [rad/s]
//  - Outputs left_rpm, right_rpm
void body_to_wheel_rpm(float v_mps, float w_rad_s,
                       float& left_rpm, float& right_rpm);

} // namespace RobotKinematics

#endif // ROBOT_KINEMATICS_H
