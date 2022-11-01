#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "SearchTree.h"
#include "VehicleModel.h"
#include "frt_custom_msgs/Map.h"
#include "frt_custom_msgs/Landmark.h"

class MapHandler
{
private:
    
    std::vector<frt_custom_msgs::Landmark*> map;
    VehicleModel* vehicleModel;

    // Parameters
    float goalBias;
    float collisionRange;
    float spawnRange;

public:
    MapHandler();
    MapHandler(VehicleModel* vehicleModel);
    //~MapHandler();
    
    // Check for offcourse
    bool isOffCourse(std::vector<std::vector<double>>* path);

    // Check for collision with lines
    bool isOnTrackEdge(std::vector<double>* vehicleState, std::vector<frt_custom_msgs::Landmark*>* cones);

    // Get random state
    std::vector<double> getRandomState();

    // Calculate goal state
    std::vector<double> calculateGoalState();

    // Update map
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

};



#endif //MAPHANDLER_H