#ifndef STEPPER_H
#define STEPPER_H

#include "../board_config.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION         LEDC_TIMER_10_BIT

// Definições das propriedades do motor NEMA 
#define WHEEL_DIAMETER_MM       73.0 // mm 
#define MICRO_STEP_RESOLUTION   32   // default microstep (can be overridden per motor)
#define STEP_RESOLUTION         1.8  // graus por passo
#define PWM_DUTY_PERCENT        10   // Legacy; not used for step width anymore
#define VEL_RPM_MAX             25   // 10% of previous max (was 250)
#define VEL_RPM_MIN             10   // 

// Safer PWM frequency bounds for step pulses
#define MAX_PWM_FREQ 3000
#define MIN_PWM_FREQ 300 
// ( RPM / Segundo ) x ( 360 / STEP_RESOLUTION ) * MICROSTEPS 
// DRV8825 @ M0/M1/M2=HIGH => 1/32 microstep. RPM2PWM maps RPM to microstep frequency accordingly.
// f = rpm/60 * (360/step_deg) * microsteps
// With step_deg=1.8 and microsteps=32: f = rpm/60 * 200 * 32 = rpm * 106.666.. Hz
// 250 rpm -> ~26.7 kHz (within 30 kHz max)
// Legacy RPM->frequency macro kept for reference; implementation now per-instance
#define RPM2PWM(rpm) (uint32_t)fmaxf(fminf(((rpm) / 60.0f) * (360.0f / STEP_RESOLUTION) * MICRO_STEP_RESOLUTION, MAX_PWM_FREQ), MIN_PWM_FREQ)

// Simple ramp in normalized units per second (|norm| in [0,1])
#ifndef ACCEL_NORM_PER_S
#define ACCEL_NORM_PER_S        1.5f
#endif

// Deadzone with hysteresis around zero to avoid flicker
#ifndef NORM_DEADZONE_ENTER
#define NORM_DEADZONE_ENTER     0.06f   // enter deadzone if |norm| <= 0.06
#endif
#ifndef NORM_DEADZONE_EXIT
#define NORM_DEADZONE_EXIT      0.08f   // exit deadzone only when |norm| >= 0.08 (hysteresis)
#endif

// Quantize normalized command to reduce update frequency near setpoint
#ifndef NORM_QUANTUM
#define NORM_QUANTUM            0.02f   // steps of 0.02 in magnitude
#endif

// Only apply new frequency if change is significant
#ifndef FREQ_APPLY_MIN_DELTA
#define FREQ_APPLY_MIN_DELTA    20.0f   // Hz
#endif

// Fixed STEP pulse high-time in microseconds for the driver (e.g., DRV8825 >=1.9us)
#ifndef STEP_PULSE_WIDTH_US
#define STEP_PULSE_WIDTH_US    2.5f
#endif

class Stepper {
public:
    enum StepperDriverType { DRIVER_A4988 = 0, DRIVER_DRV8825 = 1 };
private:
    ledc_channel_t _pwm_channel;
    gpio_num_t _pwm_pin;
    gpio_num_t _dir_pin;
    gpio_num_t _enb_pin;
    ledc_timer_t _timer;
    uint8_t _microsteps;   // microsteps per full step (e.g., 16 or 32)
    float _step_deg;

    float _rpm_min;
    float _rpm_max;
    float _rpm;          // current RPM (signed)
    float _last_freq;    // track last set frequency for safe duty calculation
    float _pulse_us;     // high-time pulse width in microseconds for STEP
    StepperDriverType _driver;

    // Ramping and direction inversion
    float _current_norm; // applied normalized command [-1,1]
    float _target_norm;  // target normalized command [-1,1]
    float _accel_norm;   // accel in norm/s
    bool  _invert_dir;   // invert physical direction if needed
    float _last_applied_norm; // previous applied value for zero-cross logic
    bool  _in_deadzone;  // current deadzone state
    float _last_freq_applied; // last frequency we pushed to hardware

    bool _cw_turn;
    bool _torque;

    void set_pwm_freq( float freq );
    float rpm_to_freq(float rpm_abs);
    void set_pwm_duty( float duty );

public:
    Stepper( gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
             uint8_t microsteps = MICRO_STEP_RESOLUTION, float pulse_width_us = STEP_PULSE_WIDTH_US, bool invert_dir = false );

    // Overload: pass driver type + microsteps (pulse width is chosen accordingly)
    Stepper( gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
             StepperDriverType driver, uint8_t microsteps, bool invert_dir = false );

    // Set desired velocity target as normalized command in [-1, 1]
    void set_velocity( float norm );
    // Update towards target with simple ramp and safe zero-cross logic
    void update( float dt_sec );
    // Immediate torque control (active low enable)
    void set_torque( bool torque ); 

    float get_velocity( void );
    bool get_torque( void );
};

#endif