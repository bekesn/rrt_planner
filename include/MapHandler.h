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
    std::vector<double> goalState;

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
    std::vector<double> getRandomState(std::vector<std::vector<double>>* path);

    // Calculate and get goal state
    void calculateGoalState();
    std::vector<double> getGoalState();

    // Update map
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

    // Visualize significant points
    void visualizePoints(visualization_msgs::MarkerArray* mArray);

    // Get closest cone by color
    frt_custom_msgs::Landmark* getClosestLandmark(frt_custom_msgs::Landmark* landmark, frt_custom_msgs::Landmark::_color_type color);

};



#endif //MAPHANDLER_H