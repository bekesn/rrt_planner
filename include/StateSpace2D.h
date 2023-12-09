#ifndef STATESPACE2D_H
#define STATESPACE2D_H

#include <memory>
#include <math.h>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include "Types.h"

using namespace std;

template<class StateSpaceVector>
class Trajectory;

class StateSpace2D
{
protected:
    float x_;
    float y_;
    float theta_;
    float vx_;

public:
    StateSpace2D();
    StateSpace2D(float x, float y, float theta, float vx);
    StateSpace2D(const StateSpace2D &original);
    //~StateSpace2D();

    // Initialize StateSpace
    static void initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam);

    // Calculate distance to target, taking into account the orientation
    float getDistToTarget(const StateSpace2D& target, const unique_ptr<RRT_PARAMETERS>& param) const;

    // Calculate distance between states, taking into account the orientation
    static float getDistOriented(const StateSpace2D& state1, const StateSpace2D& state2, const unique_ptr<RRT_PARAMETERS>& param);
    float getDistOriented(const StateSpace2D& otherState, const unique_ptr<RRT_PARAMETERS>& param) const;

    // Euclidean distance
    static float getDistEuclidean(const std::vector<float> state1, const std::vector<float> state2);
    float getDistEuclidean(const std::vector<float> otherState) const;
    float getDistEuclidean(const StateSpace2D& otherState) const;

    // Calculate angular difference in rad
    // Anticlockwise
    float getAngleToTarget(const StateSpace2D& target) const;
    float getAngleDiff(const StateSpace2D& otherState) const;

    // Define operators
    StateSpace2D operator+ (const StateSpace2D & otherState) const;
    StateSpace2D operator- (const StateSpace2D & otherState) const;
    StateSpace2D operator* (const float & multiplier) const;

    float x(void) const;
    float y(void) const;
    float theta(void) const;
    float vx(void) const;

    // Simulation to given target
    static shared_ptr<Trajectory<StateSpace2D>> simulate(const shared_ptr<StateSpace2D>& start, const shared_ptr<StateSpace2D>& goal,
                const unique_ptr<RRT_PARAMETERS>& param,  const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier = 1.0f);

    // Convert to Point for visualization
    shared_ptr<geometry_msgs::Point> toPoint(void);
};

#endif //STATESPACE2D_H