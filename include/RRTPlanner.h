#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "ros/ros.h"
#include "MapHandler.h"
#include "VehicleModel.h"
#include "SearchTree.h"
#include "frt_custom_msgs/Landmark.h"
#include <visualization_msgs/Marker.h>
#include <cereal/archives/xml.hpp>
#include <fstream>

class RRTPlanner
{
    ros::Subscriber mapSubscriber;
    ros::Subscriber poseSubscriber;
    ros::Subscriber SLAMStatusSubscriber;
    ros::Subscriber odometrySubscriber;
    std::string nodeName;

    ros::WallTimer timer;

    // Objects
    MapHandler mapHandler;
    VehicleModel vehicleModel;
    VEHICLE_PARAMETERS* vehicleParam;
    MAP_PARAMETERS* mapParam;
    GENERAL_PARAMETERS* genParam;
    CONTROL_PARAMETERS* controlParam;

    SearchTree* localRRT;
    SearchTree* globalRRT;

    enum PlannerState{
        NOMAP,
        LOCALPLANNING,
        WAITFORGLOBAL,
        GLOBALPLANNING
    };

    PlannerState state;


public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();

    // load ROS parameters
    void loadParameter(const std::string& topic, float* parameter, const float defaultValue);
    void loadParameter(const std::string& topic, int* parameter, const int defaultValue);
    void loadParameter(const std::string& topic, std::string* parameter, const std::string defaultValue);
    void loadParameters(void);

    // State machine of planner
    void stateMachine(void);
    
    // Extend searchtree by a new node
    SearchTreeNode* extend(SearchTree* rrt);

    bool rewire(SearchTree* rrt, SearchTreeNode* newNode);

    // RRT on partially discovered map
    void planLocalRRT(void);

    // RRT on fully discovered map
    void planGlobalRRT(void);

    // Handle actual path and create the loop
    // Returns true if loop is closed
    bool handleActualPath(void);

    // Timer callback
    void timerCallback(const ros::WallTimerEvent &event);

    // Visualize markers
    void visualize(SearchTree* rrt);

};



#endif //RRTPLANNER_H