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
#define VEL_RPM_MAX             250  // revert to original max
#define VEL_RPM_MIN             10   // 

// Safer PWM frequency bounds for step pulses
#define MAX_PWM_FREQ 12000
#define MIN_PWM_FREQ 1500 

// ( RPM / Segundo ) x ( 360 / STEP_RESOLUTION ) * MICROSTEPS 
// DRV8825 @ M0/M1/M2=HIGH => 1/32 microstep. RPM2PWM maps RPM to microstep frequency accordingly.
// f = rpm/60 * (360/step_deg) * microsteps
// With step_deg=1.8 and microsteps=32: f = rpm/60 * 200 * 32 = rpm * 106.666.. Hz
// 250 rpm -> ~26.7 kHz (within 30 kHz max)
// Legacy RPM->frequency macro kept for reference; implementation now per-instance
#define RPM2PWM(rpm) (uint32_t)fmaxf(fminf(((rpm) / 60.0f) * (360.0f / STEP_RESOLUTION) * MICRO_STEP_RESOLUTION, MAX_PWM_FREQ), MIN_PWM_FREQ)

// Default acceleration in RPM per second (can be overridden via build_flags: -DACCEL_RPM_PER_S=600)
#ifndef ACCEL_RPM_PER_S
#define ACCEL_RPM_PER_S       400.0f
#endif

// Fixed STEP pulse high-time in microseconds for the driver (e.g., DRV8825 >=1.9us)
#ifndef STEP_PULSE_WIDTH_US
#define STEP_PULSE_WIDTH_US    3.0f
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
    float _target_rpm;   // target RPM (signed)
    float _accel_rps;    // acceleration in RPM/s
    float _last_freq;    // track last set frequency for safe duty calculation
    float _pulse_us;     // high-time pulse width in microseconds for STEP
    StepperDriverType _driver;

    bool _cw_turn;
    bool _torque;

    void set_pwm_freq( float freq );
    float rpm_to_freq(float rpm_abs);
    void set_pwm_duty( float duty );

public:
    Stepper( gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
             uint8_t microsteps = MICRO_STEP_RESOLUTION, float pulse_width_us = STEP_PULSE_WIDTH_US );

    // Overload: pass driver type + microsteps (pulse width is chosen accordingly)
    Stepper( gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
             StepperDriverType driver, uint8_t microsteps );

    // Set desired velocity as normalized command in [-1, 1]
    void set_velocity( float norm );
    // Update the motor speed toward target using acceleration ramp
    void update( float dt_sec );
    // Immediate torque control (active low enable)
    void set_torque( bool torque ); 

    float get_velocity( void );
    bool get_torque( void );
};

#endif