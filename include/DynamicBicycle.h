#ifndef DYNAMICBICYCLE_H
#define DYNAMICBICYCLE_H

#include "KinematicBicycle.h"

class DynamicBicycle : public KinematicBicycle
{
    float r_;
    float vy_;

public:
    DynamicBicycle();
    DynamicBicycle(float x, float y, float theta, float v = 10, float delta = 0); //TODO
    DynamicBicycle(const DynamicBicycle &original);
    
    // Initialize StateSpace
    static void initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam);

    // Calculate derivative of statespace vector
    shared_ptr<DynamicBicycle> derivative(const shared_ptr<Control<DynamicBicycle>>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const;

    // Limit constrained state variables
    void limitVariables(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);

    // Calculate maximum allowed vx
    float vxLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    float vxLimitNext(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& ddelta, const float& timeStep) const;
    float vxLimitDynamic(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& delta) const;

    // Calculate maximum acceleration based on G-G diagram
    float axLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    
    // Override operators
    DynamicBicycle operator+ (const DynamicBicycle & otherState) const;
    DynamicBicycle operator- (const DynamicBicycle & otherState) const;
    DynamicBicycle operator* (const float & multiplier) const;

    // Access variables
    float v(void) const ;
    float delta(void) const ;

    // Simulation to given target
    static shared_ptr<Trajectory<DynamicBicycle>> simulate(const shared_ptr<DynamicBicycle>& start, const shared_ptr<DynamicBicycle>& goal,
        const unique_ptr<RRT_PARAMETERS>& param, const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier = 1.0f);
};

#endif //DYNAMICBICYCLE_H