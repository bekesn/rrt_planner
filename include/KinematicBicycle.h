#ifndef KINEMATICBICYCLE_H
#define KINEMATICBICYCLE_H

#include "StateSpace2D.h"
#include "Solver.h"

class KinematicBicycle : public StateSpace2D
{
protected:
    float delta_;

public:
    KinematicBicycle();
    KinematicBicycle(float x, float y, float theta, float vx = 10, float delta = 0); //TODO
    KinematicBicycle(const KinematicBicycle &original);
    
    // Initialize StateSpace
    static void initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam);

    // Calculate derivative of statespace vector
    shared_ptr<KinematicBicycle> derivative(const shared_ptr<Control<KinematicBicycle>>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const;

    // Return whether state is getting closer to goal state
    bool isGettingCloser(const shared_ptr<StateSpace2D> goalState, const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;

    // Calculate distance between states, taking into account the orientation
    float getDistOriented2(const StateSpace2D& otherState, const unique_ptr<RRT_PARAMETERS>& param) const;

    // Limit constrained state variables
    void limitVariables(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);

    // Calculate maximum allowed vx
    float vxLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    float vxLimitNext(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& ddelta, const float& timeStep) const;
    float vxLimitKinematic(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& delta) const;

    // Calculate maximum acceleration based on G-G diagram
    float axLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    
    // Override operators
    KinematicBicycle operator+ (const KinematicBicycle & otherState) const;
    KinematicBicycle operator- (const KinematicBicycle & otherState) const;
    KinematicBicycle operator* (const float & multiplier) const;

    // Access variables
    float delta(void) const ;

    // Simulation to given target
    static shared_ptr<Trajectory<KinematicBicycle>> simulate(const shared_ptr<KinematicBicycle>& start, const shared_ptr<KinematicBicycle>& goal,
        const unique_ptr<RRT_PARAMETERS>& param,  const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier = 1.0f);
};

#endif //KINEMATICBICYCLE_H