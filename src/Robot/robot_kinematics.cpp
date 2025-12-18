#include "Robot/robot_kinematics.h"

#include <math.h>

namespace RobotKinematics {

// Local constant for pi if not provided by math.h
#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

float rpm_to_rad_per_sec(float rpm) {
    // omega [rad/s] = rpm * 2*pi / 60
    return rpm * (2.0f * M_PI / 60.0f);
}

float rpm_to_linear_m_per_s(float rpm) {
    // v = omega * r
    float omega = rpm_to_rad_per_sec(rpm);
    return omega * WHEEL_RADIUS_M;
}

void wheel_rpm_to_body(float left_rpm, float right_rpm,
                       float& v_mps, float& w_rad_s) {
    // Linear velocity is average of both wheel linear speeds.
    float v_left  = rpm_to_linear_m_per_s(left_rpm);
    float v_right = rpm_to_linear_m_per_s(right_rpm);

    v_mps = 0.5f * (v_left + v_right);

    // Angular velocity (calibrated): w = ANGULAR_GAIN * (v_r - v_l) / wheel_base
    w_rad_s = ANGULAR_GAIN * (v_right - v_left) / WHEEL_BASE_M;
}

float steps_per_sec_to_rpm(float steps_per_sec) {
    // rpm = (steps/s) * 60 / steps_per_rev
    return (steps_per_sec * 60.0f) / STEPS_PER_REV;
}

void step_rates_to_body(float left_steps_per_s, float right_steps_per_s,
                        float& v_mps, float& w_rad_s) {
    float left_rpm  = steps_per_sec_to_rpm(left_steps_per_s);
    float right_rpm = steps_per_sec_to_rpm(right_steps_per_s);
    wheel_rpm_to_body(left_rpm, right_rpm, v_mps, w_rad_s);
}

void body_to_wheel_rpm(float v_mps, float w_rad_s,
                       float& left_rpm, float& right_rpm) {
    // From calibrated differential-drive kinematics:
    // Direct model: w = ANGULAR_GAIN * (v_r - v_l) / b
    // Inverse: v_r - v_l = (b / ANGULAR_GAIN) * w
    float half_span = (WHEEL_BASE_M / ANGULAR_GAIN) * 0.5f;
    float v_left  = v_mps - w_rad_s * half_span;
    float v_right = v_mps + w_rad_s * half_span;

    // Convert back to RPM: rpm = v / (2*pi*r) * 60
    float denom = 2.0f * M_PI * WHEEL_RADIUS_M;
    if (denom == 0.0f) {
        left_rpm = 0.0f;
        right_rpm = 0.0f;
        return;
    }

    left_rpm  = (v_left  / denom) * 60.0f;
    right_rpm = (v_right / denom) * 60.0f;
}

} // namespace RobotKinematics
