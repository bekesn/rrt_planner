#ifndef CONTROL_H
#define CONTROL_H

#include "Types.h"
#include "StateSpace2D.h"

class Control
{
    inline static CONTROL_PARAMETERS* controlParam;

public:

    float dv;
    float ddelta;
    float MYaw;

    Control();

    // Return Control object for controlling the vehicle
    static Control* stanleyToTarget(StateSpace2D* target);
    static Control* angleControl(StateSpace2D* target);

    // Set control parameters
    static void setParam(CONTROL_PARAMETERS* param);
};

#endif // CONTROL_H