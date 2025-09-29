#include "Stepper/stepper.h"

Stepper::Stepper(gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer)
        : _pwm_pin( pwm_pin ), _dir_pin( dir_pin ), _enb_pin( enable_pin ),
            _pwm_channel( channel ), _timer( timer ),
            _microsteps(MICRO_STEP_RESOLUTION), _step_deg(STEP_RESOLUTION),
            _rpm_min(VEL_RPM_MIN), _rpm_max(VEL_RPM_MAX), _rpm(0.0f), _target_rpm(0.0f), _accel_rps(ACCEL_RPM_PER_S), _last_freq((MIN_PWM_FREQ+MAX_PWM_FREQ)/2), 
            _cw_turn(true), _torque(false)
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
    this->set_pwm_freq( RPM2PWM(VEL_RPM_MIN) );
}

void Stepper::set_pwm_freq( float freq ) {
    if (freq < MIN_PWM_FREQ) freq = MIN_PWM_FREQ;
    if (freq > MAX_PWM_FREQ) freq = MAX_PWM_FREQ;
    ledc_set_freq( LEDC_SPEED_MODE, this->_timer, (uint32_t)(freq) );
    this->_last_freq = freq;
}

void Stepper::set_pwm_duty( float duty_percent ) {
    // Keep a constant high-time pulse by computing duty = pulse_width / period
    float freq = (this->_last_freq > 1.0f ? this->_last_freq : (float)MIN_PWM_FREQ);
    float period_s = 1.0f / freq;
    float pulse_s = STEP_PULSE_WIDTH_US * 1e-6f;
    // Bound pulse width to 90% of the period and not less than one LSB
    float duty = fminf(0.9f, fmaxf(pulse_s / period_s, 1.0f / (float)(1 << LEDC_RESOLUTION)));
    uint32_t max_duty = (1 << LEDC_RESOLUTION);
    uint32_t duty_val = (uint32_t)(duty * max_duty);
    ledc_set_duty( LEDC_SPEED_MODE, this->_pwm_channel, duty_val );
    ledc_update_duty(LEDC_SPEED_MODE, this->_pwm_channel);
}


void Stepper::set_velocity( float norm ) {
    // Apenas define o alvo; a atualização gradual ocorre em update()
    norm = fminf( fmaxf( norm, -1.0f ), 1.0f );
    float target_abs = fabsf(norm) * this->_rpm_max;
    this->_target_rpm = (norm >= 0.0f) ? target_abs : -target_abs;
}

void Stepper::update( float dt_sec ) {
    // Aproxima a velocidade atual do alvo respeitando a aceleração
    float delta = this->_target_rpm - this->_rpm;
    float max_step = this->_accel_rps * dt_sec;
    if (fabsf(delta) <= max_step) {
        this->_rpm = this->_target_rpm;
    } else {
        this->_rpm += (delta > 0.0f) ? max_step : -max_step;
    }

    // Aplica no hardware
    if (this->_rpm == 0.0f) {
        ledc_stop( LEDC_SPEED_MODE, this->_pwm_channel, false );
        return;
    }

    bool new_dir = (this->_rpm >= 0.0f);
    if (new_dir != this->_cw_turn) {
        this->_cw_turn = new_dir;
        gpio_set_level( this->_dir_pin, this->_cw_turn ? true : false );
    }

    float rpm_abs = fabsf(this->_rpm);
    this->set_pwm_freq( RPM2PWM(rpm_abs) );
    // Apply duty to keep pulse width constant at the new frequency
    this->set_pwm_duty( PWM_DUTY_PERCENT );
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
