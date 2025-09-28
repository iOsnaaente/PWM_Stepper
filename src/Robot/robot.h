#ifndef ROBOT_H
#define ROBOT_H

#include "Stepper/stepper.h"

class Robot {
public:
    Stepper& _left_motor;
    Stepper& _right_motor;

    Robot( Stepper& left_motor, Stepper& right_motor );

    void drive( float vel, float turn );
    void stop();
    // Direct wheel velocity control (normalized -1..1 each)
    void drive_wheels( float left, float right );

    // Enable/disable motor drivers (active-low enable on DRV8825)
    void set_torque(bool torque);
    bool get_torque();
};

#endif
