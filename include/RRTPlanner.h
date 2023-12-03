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
    // ROS objects
    ros::Subscriber mapSubscriber;
    ros::Subscriber blueTrackBoundarySubscriber;
    ros::Subscriber yellowTrackBoundarySubscriber;
    ros::Subscriber poseSubscriber;
    ros::Subscriber SLAMStatusSubscriber;
    ros::Subscriber odometrySubscriber;
    ros::Publisher commonPublisher;
    visualization_msgs::MarkerArray commonMArray;
    ros::WallTimer timer;

    std::string nodeName;

    // General parameters
    unique_ptr<GENERAL_PARAMETERS> genParam;

    // Objects
    shared_ptr<MapHandler> mapHandler;
    shared_ptr<VehicleModel> vehicleModel;

    unique_ptr<SearchTree> localRRT;
    unique_ptr<SearchTree> globalRRT;

    PlannerState state;


public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();

    // load ROS parameters
    void loadParameter(const string& topic, float& parameter, const float defaultValue);
    void loadParameter(const string& topic, int& parameter, const int defaultValue);
    void loadParameter(const string& topic, string& parameter, const string defaultValue);
    void loadParameters(void);

    // State machine of planner
    void stateMachine(void);
    
    // Extend searchtree by a new node
    shared_ptr<SearchTreeNode> extend(unique_ptr<SearchTree>& rrt);

    // Globally optimize tree by reorganizing edges
    bool rewire(unique_ptr<SearchTree>& rrt, shared_ptr<SearchTreeNode> newNode);

    // Locally optimize tree by reorganizing edges
    void optimizeTriangles(unique_ptr<SearchTree>& rrt);
    void optimizeTriangle(unique_ptr<SearchTree>& rrt, shared_ptr<SearchTreeNode> node);

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
    void visualize(void);

};



#endif //RRTPLANNER_H