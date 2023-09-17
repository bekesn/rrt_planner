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
    shared_ptr<std::vector<shared_ptr<SearchTreeNode>>> tree;
    shared_ptr<std::vector<shared_ptr<SearchTreeNode>>> loopClosingNodes;

    RRT_TYPE type;

public:

    // Parameter struct
    RRT_PARAMETERS* param;

    int nodeCount;
    int rewireCount;
    float pathLength;
    float pathTime;
    bool pathClosed;
    bool pathFound;
    PATH_TYPE* bestPath;

    ros::Publisher markerPublisher;
    visualization_msgs::MarkerArray markerArray;

public:

    //Constructor
    SearchTree();
    SearchTree(const VehicleModel* vehicleModel, SS_VECTOR startState, RRT_TYPE rrtType);

    //Destructor
    ~SearchTree();

    // Add child node
    // Return pointer to newly inserted node
    shared_ptr<SearchTreeNode> addChild(shared_ptr<SearchTreeNode> parentNode, SS_VECTOR state, double nodeCost);

    //Remove node
    void remove(shared_ptr<SearchTreeNode> node);

    //Get nearest node
    shared_ptr<SearchTreeNode> getNearest(const SS_VECTOR* state, float minCost = 0.0f) const;

    //Get nearby nodes
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> getNearby(shared_ptr<SearchTreeNode> node) const;

    // Decide whether almost similar state already exists
    bool alreadyInTree(const SS_VECTOR* state) const;

    // Draw tree as lines
    void visualize(void);

    // Delete tree and create new
    void init(const SS_VECTOR* startState);
    void init(PATH_TYPE* initPath);

    // Get root
    SS_VECTOR* getRoot() const;

    // Traceback to root
    PATH_TYPE* traceBackToRoot(const SS_VECTOR* goalState) const;

    // Get absolute cost to node
    float getAbsCost(const shared_ptr<SearchTreeNode>& node) const;

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached() const;

    // Rewiring node from former parent to newParent node
    void rewire(shared_ptr<SearchTreeNode> node, shared_ptr<SearchTreeNode> newParent);

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(CEREAL_NVP(tree), CEREAL_NVP(loopClosingNodes), CEREAL_NVP(type), *param, 
                    CEREAL_NVP(nodeCount), CEREAL_NVP(rewireCount), CEREAL_NVP(pathLength), CEREAL_NVP(pathTime),
                    CEREAL_NVP(pathClosed), CEREAL_NVP(pathFound), *bestPath);}
};



#endif //SEARCHTREE_H