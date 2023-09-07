#ifndef CONTROL_H
#define CONTROL_H

#include "Types.h"
#include "StateSpace2D.h"

class StateSpaceSimulated;

class Control
{
    inline static CONTROL_PARAMETERS* controlParam;

public:

    float ax;
    float ddelta;
    float MYaw;

    Control();

    // Return Control object for controlling the vehicle
    static Control* stanleyToTarget(SS_VECTOR* state, StateSpace2D* target);
    static Control* angleControl(SS_VECTOR* state, StateSpace2D* target);

    // Get random acceleration value
    static double getRandomAccel(void);

    // Set control parameters
    static void setParam(CONTROL_PARAMETERS* param);

    // Limit input
    void limitValues(void);
};

#endif // CONTROL_H