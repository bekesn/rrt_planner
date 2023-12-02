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

    // Create control inputs
    static shared_ptr<Control> control(const StateSpaceSimulated& state, const StateSpaceSimulated& target,
                                       const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep);

    // Calculate steering speed for lateral control of the vehicle
    static float thetaLateralControl(const StateSpace2D& state, const StateSpace2D& target);
    static float psiLateralControl(const StateSpace2D& state, const StateSpace2D& target);

    // Calculate a_x for longitudinal control of the vehicle
    static float getRandomAccel(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);
    static float longitudinalControl(const StateSpaceSimulated& state, const StateSpaceSimulated& target, const float& timeStep);

    // Set control parameters
    static void setParameters(const unique_ptr<CONTROL_PARAMETERS> param);

    // Get control parameters
    static unique_ptr<CONTROL_PARAMETERS>& getParameters(void);

    // Limit input
    void limitValues(const StateSpaceSimulated& state, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep);
    
    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(controlParam)), CEREAL_NVP(ax), CEREAL_NVP(ddelta), CEREAL_NVP(MYaw));}   
};

#endif // CONTROL_H