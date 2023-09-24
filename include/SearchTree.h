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
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> tree;

    RRT_TYPE type;

    float pathCost;
    int nodeCount;
    int rewireCount;
    float pathLength;
    float pathTime;
    shared_ptr<PATH_TYPE> bestPath;

public:
    bool pathFound;

    // Parameter struct
    unique_ptr<RRT_PARAMETERS> param;


    ros::Publisher markerPublisher;
    visualization_msgs::MarkerArray markerArray;

public:

    //Constructor
    SearchTree();
    SearchTree(shared_ptr<SS_VECTOR> startState, RRT_TYPE rrtType);

    //Destructor
    ~SearchTree();

    // Add child node
    // Return pointer to newly inserted node
    shared_ptr<SearchTreeNode> addChild(shared_ptr<SearchTreeNode> parentNode, shared_ptr<SS_VECTOR> state, double nodeCost);

    //Remove node
    void remove(shared_ptr<SearchTreeNode> node);

    //Get nearest node
    shared_ptr<SearchTreeNode> getNearest(const shared_ptr<SS_VECTOR>& state, float minCost = 0.0f) const;

    //Get nearby nodes
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> getNearby(shared_ptr<SearchTreeNode> node) const;

    // Decide whether almost similar state already exists
    bool alreadyInTree(const shared_ptr<SS_VECTOR>& state) const;

    // Draw tree as lines
    void visualize(void);

    // Delete tree and create new
    void init(const shared_ptr<SS_VECTOR>& startState);
    void init(shared_ptr<PATH_TYPE> initPath);

    // Get root
    shared_ptr<SS_VECTOR> getRoot() const;

    // Traceback to root from given state
    // Gets closest node to given state and returns with trajectory from root to node
    shared_ptr<PATH_TYPE> traceBackToRoot(const shared_ptr<SS_VECTOR>& goalState) const;

    // Traceback to root from given node
    // Returns with trajectory from root to node
    shared_ptr<PATH_TYPE> traceBackToRoot(const shared_ptr<SearchTreeNode>& node) const;

    // Update best path which ends at given node
    void updatePath(shared_ptr<SearchTreeNode>& endNode);

    // Create closing segment to define best loop
    // Investigates if a real loop is defined
    // Investigates if cost is decreased compared to the previous cost
    // Return whether the best loop was changed
    bool closeLoop(const shared_ptr<SearchTreeNode>& startNode, const shared_ptr<SearchTreeNode>& endNode);

    // Get absolute cost to node
    float getAbsCost(const shared_ptr<SearchTreeNode>& node) const;

    // Get best path
    shared_ptr<PATH_TYPE> getBestPath(void);

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached() const;

    // Rewiring node from former parent to newParent node
    void rewire(shared_ptr<SearchTreeNode> node, shared_ptr<SearchTreeNode> newParent);

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(CEREAL_NVP(tree), CEREAL_NVP(pathCost), CEREAL_NVP(type), CEREAL_NVP(param), 
                    CEREAL_NVP(nodeCount), CEREAL_NVP(rewireCount), CEREAL_NVP(pathLength), CEREAL_NVP(pathTime),
                    CEREAL_NVP(pathFound), CEREAL_NVP(bestPath));}
};



#endif //SEARCHTREE_H