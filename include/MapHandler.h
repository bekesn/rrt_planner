#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

class MapHandler
{
private:
    
    //vector<Landmark*> map;

public:
    MapHandler();
    ~MapHandler();
    
    // Check for collision 
    void collisionCheck();

    // Update map
    void mapCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);

};



#endif //MAPHANDLER_H