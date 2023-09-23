#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "SearchTree.h"
#include "VehicleModel.h"
#include "frt_custom_msgs/Map.h"
#include "frt_custom_msgs/Landmark.h"
#include "frt_custom_msgs/SlamStatus.h"
#include <cereal/cereal.hpp> // for defer

class MapHandler
{
private:
    
    vector<frt_custom_msgs::Landmark*> map;
    vector<frt_custom_msgs::Landmark*> blueTrackBoundary;
    vector<frt_custom_msgs::Landmark*> yellowTrackBoundary;
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

    // Check for offcourse if boundaries are not given
    // Should not be called directly
    bool isOffCourseNoBoundary(const shared_ptr<PATH_TYPE>& path) const;

    // Check for offcourse if boundaries not given
    // Should not be called directly
    bool isOffCourseWithBoundary(const shared_ptr<PATH_TYPE>& path) const;

    // Check for collision with lines
    bool isOnTrackEdge(const shared_ptr<SS_VECTOR>& vehicleState, const std::vector<frt_custom_msgs::Landmark*>* cones) const;

    // Check for collision with 1 line
    bool isOnTrackEdge(const shared_ptr<SS_VECTOR>& vehicleState, const frt_custom_msgs::Landmark* cone1, const frt_custom_msgs::Landmark* cone2) const;

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
    static bool updateLandmarkVector(const frt_custom_msgs::Map::ConstPtr &msg, vector<frt_custom_msgs::Landmark*> &vec);

    // Update SLAM status
    void SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg);

    // Visualize significant points
    void visualizePoints(visualization_msgs::MarkerArray* mArray) const;

    // Get closest cone by color
    frt_custom_msgs::Landmark* getClosestLandmark(const frt_custom_msgs::Landmark* landmark, const frt_custom_msgs::Landmark::_color_type color) const;

    // Return if map arrived
    MapHandlerState getState(void) const;

    // Return if loop is closed in SLAM
    bool isLoopClosed(void) const;

    // Archive function for cereal
    // Map is not archived
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(mapParam)),cereal::defer(CEREAL_NVP(vehicleModel)),
                                        CEREAL_NVP(goalState), CEREAL_NVP(mapReceived), CEREAL_NVP(loopClosed));}

};



#endif //MAPHANDLER_H