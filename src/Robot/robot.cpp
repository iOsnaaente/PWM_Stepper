#include "Robot/robot.h"

Robot::Robot( Stepper& left_motor, Stepper& right_motor )
    : _left_motor(left_motor), _right_motor(right_motor) {
    _left_motor.set_torque(true);
    _right_motor.set_torque(true);
}


void Robot::drive(float vel, float turn) {
    vel  = fmin( fmax( vel,  -1.0 ), 1.0 );
    turn = fmin( fmax( turn, -1.0 ), 1.0 );

    float left  = fmin( fmax( vel - turn, -1.0 ), 1.0 );
    float right = fmin( fmax( vel + turn, -1.0 ), 1.0 );

    _left_motor.set_velocity(left);
    _right_motor.set_velocity(right);
}

void Robot::drive_wheels(float left, float right) {
    left  = fmin( fmax( left,  -1.0 ), 1.0 );
    right = fmin( fmax( right, -1.0 ), 1.0 );
    _left_motor.set_velocity(left);
    _right_motor.set_velocity(right);
}


void Robot::stop() {
    _left_motor.set_velocity(0.0);
    _right_motor.set_velocity(0.0);
}