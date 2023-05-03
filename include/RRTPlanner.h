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
    std::string nodeName;

    ros::WallTimer timer;

    // Objects
    MapHandler mapHandler;
    VehicleModel vehicleModel;
    VEHICLE_PARAMETERS* vehicleParam;
    MAP_PARAMETERS* mapParam;
    GENERAL_PARAMETERS* genParam;
    CONTROL_PARAMETERS* controlParam;

    struct RRTObject{
        std::string* name;
        SearchTree* tree;

        // Parameters
        RRT_PARAMETERS* param;

        // Variables
        bool pathFound;
        bool pathClosed;
        PATH_TYPE* bestPath;

        // Visualisation
        ros::Publisher markerPublisher;
        visualization_msgs::MarkerArray markerArray;
    };

    RRTObject* localRRT;
    RRTObject* globalRRT;

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

    // Init local or global object
    void initObject(RRTObject* obj, const char* ID);

    // load ROS parameters
    void loadParameter(const std::string& topic, float* parameter, const float defaultValue);
    void loadParameter(const std::string& topic, int* parameter, const int defaultValue);
    void loadParameter(const std::string& topic, std::string* parameter, const std::string defaultValue);
    void loadParameters(void);

    // State machine of planner
    void stateMachine(void);
    
    // Extend searchtree by a new node
    SearchTreeNode* extend(RRTObject* rrt);

    bool rewire(RRTObject* rrt, SearchTreeNode* newNode);

    // RRT on partially discovered map
    void planLocalRRT(void);

    // RRT on fully discovered map
    void planGlobalRRT(void);

    // Timer callback
    void timerCallback(const ros::WallTimerEvent &event);

    // Visualize markers
    void visualize(RRTObject* rrt);

    // Visualize best path
    void visualizeBestPath(RRTObject* rrt);

};



#endif //RRTPLANNER_H