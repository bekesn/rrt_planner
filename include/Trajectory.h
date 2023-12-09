#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <visualization_msgs/Marker.h>
#include "Types.h"
#include "DynamicBicycle.h"

template<typename StateSpaceVector>
class Trajectory : public vector<shared_ptr<StateSpaceVector>>
{

    
public:

    using vector<shared_ptr<StateSpaceVector>>::push_back;

    Trajectory(/* args */);
    ~Trajectory();

    // COST FUNCTIONS
    // Cost function
    float cost(const unique_ptr<RRT_PARAMETERS>& param) const;

    // Cost according to length of trajectory
    float getDistanceCost(void) const;

    // Cost according to elapsed time
    float getTimeCost(void) const;

    // Return refined trajectory by simulating steps
    shared_ptr<Trajectory<StateSpaceVector>> getSimulated(const unique_ptr<RRT_PARAMETERS>& param, const unique_ptr<VEHICLE_PARAMETERS>& vParam);

    // Visualize
    void visualize(visualization_msgs::Marker* marker);

};


#endif //TRAJECTORY_H