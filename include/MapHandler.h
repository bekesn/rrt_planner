#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "SearchTree.h"
#include "VehicleModel.h"
#include "frt_custom_msgs/Map.h"
#include "frt_custom_msgs/Landmark.h"
#include "frt_custom_msgs/SlamStatus.h"
#include <kdtree/kdtree.hpp>

class MapHandler
{
private:
    
    vector<shared_ptr<frt_custom_msgs::Landmark>> map;
    shared_ptr<Kdtree::KdTree> mapKdTree;
    vector<shared_ptr<frt_custom_msgs::Landmark>> blueTrackBoundary;
    vector<shared_ptr<frt_custom_msgs::Landmark>> yellowTrackBoundary;
    shared_ptr<VehicleModel> vehicleModel;
    shared_ptr<SS_VECTOR> goalState;
    bool mapReceived;
    bool blueTrackBoundaryReceived;
    bool yellowTrackBoundaryReceived;
    bool loopClosed;
    MapHandlerState state;

    unique_ptr<MAP_PARAMETERS> mapParam;

public:
    MapHandler(void);
    MapHandler(unique_ptr<MAP_PARAMETERS> param, const shared_ptr<VehicleModel> vehicleModel);
    //~MapHandler();
    
    // Check for offcourse
    bool isOffCourse(const shared_ptr<PATH_TYPE>& path) const;

    // Upsample cones
    void upSample(void);

    // Get random state
    shared_ptr<SS_VECTOR> getRandomState(const shared_ptr<PATH_TYPE>& path, const unique_ptr<RRT_PARAMETERS>& param) const;

    // Get parameters
    unique_ptr<MAP_PARAMETERS>& getParameters(void);

    // Calculate and get goal state
    void calculateGoalState(void);
    shared_ptr<SS_VECTOR> getGoalState(void);

    // Update map
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

    // Update blue boundary (from path planner)
    void blueTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg);

    // Update yellow boundary (from path planner)
    void yellowTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg);

    // Update given Landmark vector
    static bool updateLandmarkVector(const frt_custom_msgs::Map::ConstPtr &msg, vector<shared_ptr<frt_custom_msgs::Landmark>> &vec);

    // Update SLAM status
    void SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg);

    // Visualize significant points
    void visualize(visualization_msgs::MarkerArray* mArray) const;

    // Get closest cone by color
    shared_ptr<frt_custom_msgs::Landmark> getClosestLandmark(const shared_ptr<frt_custom_msgs::Landmark>& landmark, const frt_custom_msgs::Landmark::_color_type& color) const;

    // Return if map arrived
    MapHandlerState getState(void) const;

    // Return if loop is closed in SLAM
    bool isLoopClosed(void) const;

};



#endif //MAPHANDLER_H