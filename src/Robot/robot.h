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
};

#endif
