#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "ros/ros.h"
#include "Control.h"
#include "MapHandler.h"
#include "Vehicle.h"
#include "SearchTree.h"
#include <visualization_msgs/Marker.h>

template<class StateSpaceVector>
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
    shared_ptr<MapHandler<StateSpaceVector>> mapHandler;
    shared_ptr<Vehicle<StateSpaceVector>> vehicle;

    unique_ptr<SearchTree<StateSpaceVector>> localRRT;
    unique_ptr<SearchTree<StateSpaceVector>> globalRRT;

    PlannerState state;


public:
    RRTPlanner(int argc, char** argv);
    //~RRTPlanner();

    // load ROS parameters
    void loadParameter(const string& topic, float& parameter, const float defaultValue);
    void loadParameter(const string& topic, int& parameter, const int defaultValue);
    void loadParameter(const string& topic, string& parameter, const string defaultValue);
    void loadParameters(unique_ptr<CONTROL_PARAMETERS>& controlParam);

    // State machine of planner
    void stateMachine(void);
    
    // Extend searchtree by a new node
    shared_ptr<SearchTreeNode<StateSpaceVector>> extend(unique_ptr<SearchTree<StateSpaceVector>>& rrt);

    // Globally optimize tree by reorganizing edges
    bool rewire(unique_ptr<SearchTree<StateSpaceVector>>& rrt, shared_ptr<SearchTreeNode<StateSpaceVector>> newNode);

    // Locally optimize tree by reorganizing edges
    void optimizeTriangles(unique_ptr<SearchTree<StateSpaceVector>>& rrt);
    void optimizeTriangle(unique_ptr<SearchTree<StateSpaceVector>>& rrt, shared_ptr<SearchTreeNode<StateSpaceVector>> node);

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