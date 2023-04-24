#ifndef CONTROL_H
#define CONTROL_H

#include "Types.h"
#include "StateSpace2D.h"

class Control
{
    float throttle;
    float steeringAngle;
    float MYaw;

    CONTROL_PARAMETERS* param;


public:
    Control();
    Control(CONTROL_PARAMETERS* par);

    static Control* StanleyToTarget(StateSpace2D* target);
};

#endif // CONTROL_H