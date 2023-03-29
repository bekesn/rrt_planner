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

    typedef enum {
        NOMAP,
        LOCALPLANNING,
        WAITFORGLOBAL,
        GLOBALPLANNING
    }PlannerState;

    // Variables
    bool pathFound;
    bool pathClosed;
    PATH_TYPE* bestPath;
    PlannerState state;

    // Parameters
    RRT_PARAMETERS* param;

public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();

    // load ROS parameters
    void loadParameters(void);

    // State machine of planner
    void stateMachine(void);
    
    // Extend searchtree by a new node
    SearchTreeNode* extend(void);

    bool rewire(SearchTreeNode* newNode);

    // RRT on partially discovered map
    void planLocalRRT(void);

    // RRT on fully discovered map
    void planGlobalRRT(void);

    // Timer callback
    void timerCallback(const ros::WallTimerEvent &event);

    // Visualize markers
    void visualize(void);

    // Visualize best path
    void visualizeBestPath(visualization_msgs::MarkerArray* mArray);

};



#endif //RRTPLANNER_H