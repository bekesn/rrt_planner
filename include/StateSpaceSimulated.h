#ifndef STATESPACESIMULATED_H
#define STATESPACESIMULATED_H

#include "StateSpace2D.h"
#include "Control.h"
#include <cereal/types/base_class.hpp>
#include <ros/ros.h>

class StateSpaceSimulated : public StateSpace2D
{
    float v_;
    float delta_;
public:
    StateSpaceSimulated();
    StateSpaceSimulated(float x, float y, float theta, float v = 10, float delta = 0); //TODO
    StateSpaceSimulated(const StateSpaceSimulated &original);
    //~StateSpace();

    // Calculate derivative of statespace vector
    shared_ptr<StateSpaceSimulated> derivative(const shared_ptr<Control>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const;

    // Return whether state is getting closer to goal state
    bool isGettingCloser(const shared_ptr<StateSpace2D> goalState, const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;

    // Limit constrained state variables
    void limitVariables(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);

    // Calculate maximum allowed vx
    float vxLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    float vxLimitNext(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& ddelta, const float& timeStep) const;
    float vxLimitKinematic(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& delta) const;

    // Calculate maximum acceleration based on G-G diagram
    float axLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const;
    
    // Override operators
    StateSpaceSimulated operator+ (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator- (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator* (const float & multiplier) const;

    // Access variables
    float v(void) const ;
    float delta(void) const ;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::base_class<StateSpace2D> ( this ), CEREAL_NVP(v_), CEREAL_NVP(delta_));}

};

#endif //STATESPACE_H