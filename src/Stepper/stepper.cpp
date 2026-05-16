#include "Stepper/stepper.h"

namespace {
inline int enable_level_for(Stepper::StepperDriverType driver, bool torque_enabled) {
    // Convert logical torque state into electrical EN pin level.
    if (driver == Stepper::DRIVER_DRV8825) {
        const int enabled_level  = DRV8825_ENABLE_ACTIVE_HIGH ? 1 : 0;
        const int disabled_level = DRV8825_ENABLE_ACTIVE_HIGH ? 0 : 1;
        return torque_enabled ? enabled_level : disabled_level;
    }
    const int enabled_level  = A4988_ENABLE_ACTIVE_HIGH ? 1 : 0;
    const int disabled_level = A4988_ENABLE_ACTIVE_HIGH ? 0 : 1;
    return torque_enabled ? enabled_level : disabled_level;
}
}

Stepper::Stepper(gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
                                 uint8_t microsteps, float pulse_width_us, bool invert_dir)
        : _pwm_pin( pwm_pin ), _dir_pin( dir_pin ), _enb_pin( enable_pin ),
            _pwm_channel( channel ), _timer( timer ),
            _microsteps(microsteps), _step_deg(STEP_RESOLUTION),
            _rpm_min(VEL_RPM_MIN), _rpm_max(VEL_RPM_MAX), _rpm(0.0f), _last_freq((MIN_PWM_FREQ+MAX_PWM_FREQ)/2), 
            _pulse_us(pulse_width_us), _driver(DRIVER_DRV8825), _cw_turn(true), _torque(false),
            _current_norm(0.0f), _target_norm(0.0f), _accel_norm(ACCEL_NORM_PER_S), _invert_dir(invert_dir),
            _last_applied_norm(0.0f), _in_deadzone(true), _last_freq_applied(0.0f)
{
    // Configura pino de direção
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << _dir_pin) | (1ULL << _enb_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config( &dir_cfg );
    
    // Configura timer PWM
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_SPEED_MODE,
        .duty_resolution  = LEDC_RESOLUTION,
        .timer_num        = _timer,
        .freq_hz          = (MIN_PWM_FREQ+MAX_PWM_FREQ)/2,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config( &timer_cfg );
    
    // Configura canal PWM
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = _pwm_pin,
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = _pwm_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = _timer,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config( &channel_cfg );
    
    // Seta a diração de giro 
    gpio_set_level( this->_dir_pin, this->_cw_turn );

    // Apply default torque state to EN pin using selected driver polarity.
    gpio_set_level( this->_enb_pin, enable_level_for(this->_driver, this->_torque) );
}

Stepper::Stepper(gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
                 StepperDriverType driver, uint8_t microsteps, bool invert_dir)
    : Stepper(pwm_pin, dir_pin, enable_pin, channel, timer,
              microsteps,
              // Choose default pulse width per driver
              (driver == DRIVER_A4988 ? 2.0f : 3.0f), invert_dir)
{
    this->_driver = driver;
    // Re-apply EN level because delegating ctor initializes _driver as DRV8825.
    gpio_set_level( this->_enb_pin, enable_level_for(this->_driver, this->_torque) );
}

void Stepper::set_pwm_freq( float freq ) {
    if (freq < MIN_PWM_FREQ) freq = MIN_PWM_FREQ;
    if (freq > MAX_PWM_FREQ) freq = MAX_PWM_FREQ;
    ledc_set_freq( LEDC_SPEED_MODE, this->_timer, (uint32_t)(freq) );
    this->_last_freq = freq;
}

float Stepper::rpm_to_freq(float rpm_abs) {
    // f = rpm/60 * steps_per_rev * microsteps
    float steps_per_rev = 360.0f / this->_step_deg; // e.g., 200
    float f = (rpm_abs / 60.0f) * steps_per_rev * (float)this->_microsteps;
    if (f < MIN_PWM_FREQ) f = MIN_PWM_FREQ;
    if (f > MAX_PWM_FREQ) f = MAX_PWM_FREQ;
    return f;
}

void Stepper::set_pwm_duty( float duty_percent ) {
    (void)duty_percent;
    // Known-good behavior for A4988-style STEP input: use a 50% duty square wave.
    // This produces wide pulses (like the user's reference sketch) and avoids extremely narrow highs.
    uint32_t max_duty = (1 << LEDC_RESOLUTION);
    uint32_t duty_val = max_duty / 2;
    ledc_set_duty( LEDC_SPEED_MODE, this->_pwm_channel, duty_val );
    ledc_update_duty(LEDC_SPEED_MODE, this->_pwm_channel);
}


void Stepper::set_velocity( float norm ) {
    // Store target command; update() will ramp _current_norm toward this
    _target_norm = fminf( fmaxf( norm, -1.0f ), 1.0f );
}

void Stepper::update( float dt_sec ) {
    if (dt_sec <= 0.0f) {
        dt_sec = 0.0f;
    }

    // Apply acceleration-limited ramp from _current_norm toward _target_norm
    if (dt_sec > 0.0f && _accel_norm > 0.0f) {
        float maxStep = _accel_norm * dt_sec;
        float delta   = _target_norm - _current_norm;
        if (delta > maxStep)       delta = maxStep;
        else if (delta < -maxStep) delta = -maxStep;
        _current_norm += delta;
    } else {
        // No valid dt or accel -> jump directly (should be rare)
        _current_norm = _target_norm;
    }

    float applied = _current_norm;

    // Small epsilon deadzone to fully stop when near zero
    static const float eps = 1e-3f;
    if (fabsf(applied) <= eps) {
        this->_rpm = 0.0f;
        ledc_stop(LEDC_SPEED_MODE, this->_pwm_channel, false);
        return;
    }

    bool dir = (applied >= 0.0f);
    if (_invert_dir) dir = !dir;
    if (dir != this->_cw_turn) {
        this->_cw_turn = dir;
        gpio_set_level(this->_dir_pin, this->_cw_turn ? 1 : 0);
    }

    float mag = fabsf(applied);
    // Map normalized magnitude [0,1] into a much lower, configured RPM band
    float rpm_abs = _rpm_min + (_rpm_max - _rpm_min) * mag; // e.g., 10..25 RPM
    float freq = rpm_to_freq(rpm_abs);

    // Apply frequency and update reported RPM
    this->set_pwm_freq(freq);
    this->_rpm = dir ? rpm_abs : -rpm_abs;
    this->set_pwm_duty(PWM_DUTY_PERCENT);
}

void Stepper::set_max_accel_norm( float accel_norm ) {
    if (accel_norm < 0.0f) accel_norm = -accel_norm;
    _accel_norm = accel_norm;
}

void Stepper::set_torque(bool torque ) {
    this->_torque = torque;
    gpio_set_level( this->_enb_pin, enable_level_for(this->_driver, this->_torque) );
}

float Stepper::get_velocity(void) {
    return this->_rpm;
}

bool Stepper::get_torque(void) {
    return this->_torque;
}
