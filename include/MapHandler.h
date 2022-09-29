#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "ros/ros.h"
#include "frt_custom_msgs/Map.h"

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
    void mapCallback(const frt_custom_msgs::Map::ConstPtr &msg);

};



#endif //MAPHANDLER_H