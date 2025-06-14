#include "Stepper/stepper.h"

Stepper::Stepper(gpio_num_t pwm_pin, gpio_num_t dir_pin, gpio_num_t enable_pin, ledc_channel_t channel, ledc_timer_t timer)
    : _pwm_pin( pwm_pin ), _dir_pin( dir_pin ), _enb_pin( enable_pin ),
      _pwm_channel( channel ), _timer( timer ),
      _microsteps(MICRO_STEP_RESOLUTION), _step_deg(STEP_RESOLUTION),
      _rpm_min(VEL_RPM_MIN), _rpm_max(VEL_RPM_MAX), _rpm(VEL_RPM_MIN), 
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
    ledc_set_freq( LEDC_SPEED_MODE, this->_timer, (uint32_t)(freq) );
}

void Stepper::set_pwm_duty( float duty_percent ) {
    uint32_t max_duty = (1 << LEDC_RESOLUTION);
    uint32_t duty_val = (uint32_t)(( duty_percent / 100.0f) * max_duty );
    ledc_set_duty( LEDC_SPEED_MODE, this->_pwm_channel, duty_val );
    ledc_update_duty(LEDC_SPEED_MODE, this->_pwm_channel);
}


void Stepper::set_velocity( float rpm ) {
    // Se 0.0 então para 
    if ( rpm == 0.0 ) {
        ledc_stop( LEDC_SPEED_MODE, this->_pwm_channel, false );
        this->_rpm = 0.0;
        return;
    }        
    
    // Clamp em [-1.0, 1.0] ou parado 
    rpm = fmin( fmax( rpm, -1.0 ), 1.0 );
    
    // Define direção
    this->_cw_turn = ( rpm  >= 0.0);
    gpio_set_level( this->_dir_pin, this->_cw_turn ? true : false );
    
    // Calcula RPM real a partir do valor normalizado
    float rpm_abs = fabs( rpm ) * this->_rpm_max;
    this->_rpm = this->_cw_turn ? rpm_abs : -rpm_abs;
    this->set_pwm_freq( RPM2PWM(rpm_abs) );

    // Religa o Duty para garantir que saiu do estado STOP 
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
