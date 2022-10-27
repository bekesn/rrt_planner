#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    // Init objects
    vehicleModel = VehicleModel(&VehicleModel::getDistEuclidean, &VehicleModel::simulateHolonomic);
    mapHandler = MapHandler(&vehicleModel);
    sTree = SearchTree(&vehicleModel, {10.0, 0.0, 0.0});

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_viz", 10);


    timer = nh.createWallTimer(ros::WallDuration(0.5), &RRTPlanner::timerCallback, this);

    
    ros::spin();
}


void RRTPlanner::extend()
{
    bool offCourse;
    std::vector<double> randState;
    std::vector<double> newState;
    std::vector<std::vector<double>>* path;

    // Get random state
    randState = mapHandler.getRandomState();

    // Get nearest node
    SearchTreeNode* nearest = sTree.getNearest(randState);

    // Simulate movement towards new state
    path = vehicleModel.simulateToTarget(nearest->getState(), randState);

    // Check for offCourse
    offCourse = mapHandler.isOffCourse(path);

    // Add node
    if (!offCourse && (path->size() > 0))
    {
        newState = path->back();
        sTree.addChild(nearest, newState);
    }
}

void RRTPlanner::planOpenTrackRRT()
{
    sTree.reset({10.0, 0.0, 0.0});

    // TODO
    for(int i = 0; i < 300; i++)
    {
        extend();
    }
    visualize();
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::planClosedTrackRRT()
{
    sTree.reset({10.0, 0.0, 0.0});

    // TODO
    for(int i = 0; i < 300; i++)
    {
        extend();
    }
    visualize();
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    planOpenTrackRRT();
}

void RRTPlanner::visualize()
{
    sTree.drawTree(&markerArray);
    markerPublisher.publish(markerArray);
}

int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}
