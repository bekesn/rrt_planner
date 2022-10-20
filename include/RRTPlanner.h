#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "ros/ros.h"
#include "MapHandler.h"
#include "VehicleModel.h"
#include "SearchTree.h"
#include "frt_custom_msgs/Landmark.h"
#include <visualization_msgs/Marker.h>

class RRTPlanner
{
    ros::Subscriber mapSubscriber;
    ros::Subscriber poseSubscriber;

    ros::Publisher markerPublisher;

    ros::WallTimer timer;


    MapHandler mapHandler;
    VehicleModel vehicleModel;
    SearchTree sTree;

    visualization_msgs::MarkerArray markerArray;

public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();
    
    // Extend searchtree by a new node
    void extend();

    // Visualize markers
    void visualize(const ros::WallTimerEvent &event);

};



#endif //RRTPLANNER_H