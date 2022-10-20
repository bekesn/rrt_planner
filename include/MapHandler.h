#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "SearchTree.h"
#include "frt_custom_msgs/Map.h"
#include "frt_custom_msgs/Landmark.h"

class MapHandler
{
private:
    
    std::vector<frt_custom_msgs::Landmark*> map;

    // Parameters
    double goalBias;

public:
    MapHandler();
    //~MapHandler();
    
    // Check for offcourse
    bool isOffCourse(std::vector<std::vector<double>>* path);

    // Get random state
    std::vector<double> getRandomState();

    // Calculate goal state
    std::vector<double> calculateGoalState();

    // Update map
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

};



#endif //MAPHANDLER_H