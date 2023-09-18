#ifndef CONTROL_H
#define CONTROL_H

#include "Types.h"
#include "StateSpace2D.h"

class StateSpaceSimulated;

class Control
{
    static unique_ptr<CONTROL_PARAMETERS> controlParam;

public:

    float ax;
    float ddelta;
    float MYaw;

    Control();

    // Return Control object for controlling the vehicle
    static shared_ptr<Control> stanleyToTarget(const SS_VECTOR& state, const StateSpace2D& target);
    static shared_ptr<Control> angleControl(const SS_VECTOR& state, const StateSpace2D& target);

    // Get random acceleration value
    static double getRandomAccel(void);

    // Set control parameters
    static void setParameters(const unique_ptr<CONTROL_PARAMETERS> param);

    // Get control parameters
    static unique_ptr<CONTROL_PARAMETERS>& getParameters(void);

    // Limit input
    void limitValues(void);
    
    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(controlParam)), CEREAL_NVP(ax), CEREAL_NVP(ddelta), CEREAL_NVP(MYaw));}   
};

#endif // CONTROL_H