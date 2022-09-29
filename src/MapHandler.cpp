#include "MapHandler.h"

MapHandler::MapHandler()
{

}

MapHandler::~MapHandler()
{
    
}

void MapHandler::mapCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    ROS_INFO_STREAM(msg);
}