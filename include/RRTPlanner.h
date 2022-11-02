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

    bool pathFound;
    std::vector<std::vector<double>>* bestPath;

public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();
    
    // Extend searchtree by a new node
    bool extend();

    bool rewire(SearchTreeNode* newNode);

    // RRT on partially discovered map
    void planOpenTrackRRT();

    // RRT on fully discovered map
    void planClosedTrackRRT();

    // Timer callback
    void timerCallback(const ros::WallTimerEvent &event);

    // Visualize markers
    void visualize();

    // Visualize best path
    void visualizeBestPath(visualization_msgs::MarkerArray* mArray);

};



#endif //RRTPLANNER_H