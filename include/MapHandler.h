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
    
    std::vector<frt_custom_msgs::Landmark*> map;
    VehicleModel* vehicleModel;
    SS_VECTOR goalState;
    bool mapReceived;
    bool loopClosed;

    unique_ptr<MAP_PARAMETERS> mapParam;

public:
    MapHandler(void);
    MapHandler(unique_ptr<MAP_PARAMETERS> param, VehicleModel* vehicleModel);
    //~MapHandler();
    
    // Check for offcourse
    bool isOffCourse(shared_ptr<PATH_TYPE> path, unique_ptr<RRT_PARAMETERS>& param);

    // Check for collision with lines
    bool isOnTrackEdge(SS_VECTOR* vehicleState, std::vector<frt_custom_msgs::Landmark*>* cones, unique_ptr<RRT_PARAMETERS>& param);

    // Get random state
    SS_VECTOR* getRandomState(shared_ptr<PATH_TYPE> path, unique_ptr<RRT_PARAMETERS>& param);

    // Get parameters
    unique_ptr<MAP_PARAMETERS>& getParameters(void);

    // Calculate and get goal state
    void calculateGoalState();
    SS_VECTOR getGoalState();

    // Update map
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

    // Update SLAM status
    void SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg);

    // Visualize significant points
    void visualizePoints(visualization_msgs::MarkerArray* mArray);

    // Get closest cone by color
    frt_custom_msgs::Landmark* getClosestLandmark(frt_custom_msgs::Landmark* landmark, frt_custom_msgs::Landmark::_color_type color);

    // Return if map arrived
    bool hasMap(void);

    // Return if loop is closed in SLAM
    bool isLoopClosed(void);

    // Archive function for cereal
    // Map is not archived
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(mapParam)),cereal::defer(CEREAL_NVP(vehicleModel)),
                                        CEREAL_NVP(goalState), CEREAL_NVP(mapReceived), CEREAL_NVP(loopClosed));}

};



#endif //MAPHANDLER_H