#ifndef CONTROL_H
#define CONTROL_H

#include "Types.h"
#include "StateSpace2D.h"
#include "DynamicBicycle.h"

template<class StateSpaceVector>
class Control
{
    static unique_ptr<CONTROL_PARAMETERS> controlParam;

public:

    float ax;
    float ddelta;
    float MYaw;

    Control();

    // Create control inputs
    static shared_ptr<Control> control(const StateSpaceVector& state, const StateSpaceVector& target,
                                       const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep);

    // Calculate steering speed for lateral control of the vehicle
    static float thetaLateralControl(const StateSpace2D& state, const StateSpace2D& target, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);
    static float psiLateralControl(const StateSpace2D& state, const StateSpace2D& target, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);

    // Calculate a_x for longitudinal control of the vehicle
    static float getRandomAccel(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);
    static float longitudinalControl(const StateSpaceVector& state, const StateSpaceVector& target, const float& timeStep);

    // Set control parameters
    static void setParameters(const unique_ptr<CONTROL_PARAMETERS> param);

    // Get control parameters
    static unique_ptr<CONTROL_PARAMETERS>& getParameters(void);

    // Limit input
    void limitValues(const StateSpaceVector& state, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep);  
};

#endif // CONTROL_H