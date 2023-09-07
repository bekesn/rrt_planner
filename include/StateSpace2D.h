#ifndef STATESPACE2D_H
#define STATESPACE2D_H

#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include "Types.h"


class StateSpace2D
{
protected:
    float x_;
    float y_;
    float theta_;

public:
    StateSpace2D();
    StateSpace2D(float x, float y, float theta);
    StateSpace2D(const StateSpace2D &original);
    //~StateSpace2D();

    // Calculate distance to target
    float distanceToTarget(const StateSpace2D* target, const RRT_PARAMETERS* param);

    // Euclidean distance
    static double getDistEuclidean(const std::vector<float> start, const std::vector<float> goal);
    float getDistEuclidean(const std::vector<float> target);
    float getDistEuclidean(const StateSpace2D* target);

    // Calculate angular difference in rad
    // Anticlockwise
    double angleToTarget(const StateSpace2D* target);

    // Define operators
    StateSpace2D operator+ (const StateSpace2D & otherState) const;
    StateSpace2D operator- (const StateSpace2D & otherState) const;
    StateSpace2D operator* (const float & multiplier) const;

    float x(void);
    float y(void);
    float theta(void);
    
};

#endif //STATESPACE2D_H