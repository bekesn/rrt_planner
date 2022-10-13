#ifndef SEARCHTREE_H
#define SEARCHTREE_H

#include <vector>
#include <SearchTreeNode.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <VehicleModel.h>

class SearchTree
{
private:
    std::vector<SearchTreeNode> *tree;
    double (VehicleModel::*distanceFunction)(std::vector<double>, std::vector<double>);
    VehicleModel* vehicle;
    int maxNumOfNodes = 40;

public:

    //Constructor
    SearchTree();
    SearchTree(VehicleModel* vehicleModel, std::vector<double> startState);

    //Destructor
    ~SearchTree();

    //Add child node
    void addChild(SearchTreeNode* parentNode, std::vector<double> state);

    //Remove node
    void remove(SearchTreeNode* node);

    //Get nearest node
    SearchTreeNode* getNearest(std::vector<double> state);

    //Get nearby nodes
    std::vector<SearchTreeNode*> getNearby(std::vector<double> state, double maxDist);

    //Set distance metric
    void setDistanceMetric(double (VehicleModel::*distanceMetric)(std::vector<double>, std::vector<double>));

    // Draw tree as lines
    void drawTree(visualization_msgs::MarkerArray* markerArray);

};



#endif //SEARCHTREE_H