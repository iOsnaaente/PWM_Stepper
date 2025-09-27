#ifndef STEPPER_H
#define STEPPER_H

#include "../board_config.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION         LEDC_TIMER_10_BIT

// Definições das propriedades do motor NEMA 
#define WHEEL_DIAMETER_MM       73.0 // mm 
#define MICRO_STEP_RESOLUTION   32   // 1/32 = passo/microPasso 
#define STEP_RESOLUTION         1.8  // graus por passo
#define PWM_DUTY_PERCENT        10   // Base duty; cpp enforces >= ~2us pulse automatically
#define VEL_RPM_MAX             250  // revert to original max
#define VEL_RPM_MIN             10   // 

#define MAX_PWM_FREQ 30000
#define MIN_PWM_FREQ 3000 

// ( RPM / Segundo ) x ( 360 / STEP_RESOLUTION ) * MICROSTEPS 
// DRV8825 @ M0/M1/M2=HIGH => 1/32 microstep. RPM2PWM maps RPM to microstep frequency accordingly.
// f = rpm/60 * (360/step_deg) * microsteps
// With step_deg=1.8 and microsteps=32: f = rpm/60 * 200 * 32 = rpm * 106.666.. Hz
// 250 rpm -> ~26.7 kHz (within 30 kHz max)
#define RPM2PWM(rpm) (uint32_t)fmaxf(fminf(((rpm) / 60.0f) * (360.0f / STEP_RESOLUTION) * MICRO_STEP_RESOLUTION, MAX_PWM_FREQ), MIN_PWM_FREQ)

class Stepper {
private:
    ledc_channel_t _pwm_channel;
    gpio_num_t _pwm_pin;
    gpio_num_t _dir_pin;
    gpio_num_t _enb_pin;
    ledc_timer_t _timer;
    uint8_t _microsteps;
    float _step_deg;

    float _rpm_min;
    float _rpm_max;
    float _rpm;
    float _last_freq; // track last set frequency for safe duty calculation

    bool _cw_turn;
    bool _torque;

    void set_pwm_freq( float freq ); 
    void set_pwm_duty( float duty );

public:
    Stepper( gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer );

    void set_velocity( float rpm );
    void set_torque( bool torque ); 

    float get_velocity( void );
    bool get_torque( void );
};

#endif