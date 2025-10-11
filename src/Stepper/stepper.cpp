#include "Stepper/stepper.h"

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

    // Seta o Torque dos motores 
    gpio_set_level( this->_enb_pin, this->_torque );

    // Seta as configurações de PWM
    this->set_pwm_duty( PWM_DUTY_PERCENT );
    this->set_pwm_freq( rpm_to_freq(VEL_RPM_MIN) );
}

Stepper::Stepper(gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer,
                 StepperDriverType driver, uint8_t microsteps, bool invert_dir)
    : Stepper(pwm_pin, dir_pin, enable_pin, channel, timer,
              microsteps,
              // Choose default pulse width per driver
              (driver == DRIVER_A4988 ? 2.0f : 3.0f), invert_dir)
{
    this->_driver = driver;
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
    // Keep a constant high-time pulse by computing duty = pulse_width / period
    float freq = (this->_last_freq > 1.0f ? this->_last_freq : (float)MIN_PWM_FREQ);
    float period_s = 1.0f / freq;
    float pulse_s = this->_pulse_us * 1e-6f;
    // Bound pulse width to 90% of the period and not less than one LSB
    float duty = fminf(0.9f, fmaxf(pulse_s / period_s, 1.0f / (float)(1 << LEDC_RESOLUTION)));
    uint32_t max_duty = (1 << LEDC_RESOLUTION);
    uint32_t duty_val = (uint32_t)(duty * max_duty);
    ledc_set_duty( LEDC_SPEED_MODE, this->_pwm_channel, duty_val );
    ledc_update_duty(LEDC_SPEED_MODE, this->_pwm_channel);
}


void Stepper::set_velocity( float norm ) {
    // Set new target; actual applied value moves in update()
    _target_norm = fminf( fmaxf( norm, -1.0f ), 1.0f );
}

void Stepper::update( float dt_sec ) {
    // Hysteresis deadzone entry/exit
    float tgt_mag = fabsf(_target_norm);
    if (_in_deadzone) {
        if (tgt_mag >= NORM_DEADZONE_EXIT) _in_deadzone = false;
    } else {
        if (tgt_mag <= NORM_DEADZONE_ENTER) {
            // Entering deadzone: force true stop and reset ramp memory
            _in_deadzone = true;
            _current_norm = 0.0f;
            _last_applied_norm = 0.0f;
            this->_rpm = 0.0f;
            ledc_stop(LEDC_SPEED_MODE, this->_pwm_channel, false);
            return; // nothing else to do this cycle
        }
    }

    // Move current_norm toward target_norm with max delta = accel * dt
    float max_delta = _accel_norm * dt_sec;
    float delta = _target_norm - _current_norm;
    if (fabsf(delta) > max_delta) {
        _current_norm += (delta > 0 ? max_delta : -max_delta);
    } else {
        _current_norm = _target_norm;
    }

    // Quantize to reduce chattering
    float applied = _current_norm;
    if (!_in_deadzone) {
        float sign = applied >= 0 ? 1.0f : -1.0f;
        float magq = floorf(fabsf(applied) / NORM_QUANTUM + 0.5f) * NORM_QUANTUM; // round to quantum
        if (magq > 1.0f) magq = 1.0f;
        applied = sign * magq;
    }

    // Zero-cross handling: do not overshoot through zero; approach zero, then change DIR after sign flips
    static const float eps = 1e-3f;
    if (_last_applied_norm > eps && _target_norm < -eps) {
        // We were positive and target is negative -> enforce monotonic decay to zero
        applied = fmaxf(0.0f, applied);
    } else if (_last_applied_norm < -eps && _target_norm > eps) {
        // We were negative and target is positive
        applied = fminf(0.0f, applied);
    }

    // Apply to hardware
    if (fabsf(applied) <= eps) {
        this->_rpm = 0.0f;
        ledc_stop(LEDC_SPEED_MODE, this->_pwm_channel, false);
        _current_norm = 0.0f;
        _last_applied_norm = 0.0f;
        return;
    }

    bool dir = (applied >= 0.0f);
    if (_invert_dir) dir = !dir;
    if (dir != this->_cw_turn) {
        this->_cw_turn = dir;
        gpio_set_level(this->_dir_pin, this->_cw_turn ? 1 : 0);
    }

    float mag = fabsf(applied);
    float freq = MIN_PWM_FREQ + (MAX_PWM_FREQ - MIN_PWM_FREQ) * mag;
    if (fabsf(freq - _last_freq_applied) >= FREQ_APPLY_MIN_DELTA) {
        this->set_pwm_freq(freq);
        _last_freq_applied = _last_freq; // set_pwm_freq updates _last_freq to actual
        // Update derived RPM for external reads
        float steps_per_rev = 360.0f / this->_step_deg;
        float rpm_abs = (freq / (steps_per_rev * (float)this->_microsteps)) * 60.0f;
        this->_rpm = dir ? rpm_abs : -rpm_abs;
        this->set_pwm_duty(PWM_DUTY_PERCENT);
    }
    _last_applied_norm = applied;
}

void Stepper::set_torque(bool torque ) {
    this->_torque = torque;
    gpio_set_level( this->_enb_pin, torque ? false : true );
}

float Stepper::get_velocity(void) {
    return this->_rpm;
}

bool Stepper::get_torque(void) {
    return this->_torque;
}
