#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <ros/ros.h>
#include <iterator>
#include "Types.h"
#include "StateSpaceSimulated.h"
#include "Trajectory.h"
#include <cereal/cereal.hpp> // for defer
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

using namespace std;

class SearchTreeNode
{
private:
    shared_ptr<SearchTreeNode> parentNode;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> childNodes;
    SS_VECTOR state;
    float cost;
    bool isRoot;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode &original);
    SearchTreeNode(shared_ptr<SearchTreeNode> parent, SS_VECTOR stateSpace, double nodeCost);

    // Destructor
    ~SearchTreeNode();

    // Add child node
    void addChild(shared_ptr<SearchTreeNode> childNode);

    // Remove child node
    void removeChild(shared_ptr<SearchTreeNode> childNode);

    // Get and change parent
    shared_ptr<SearchTreeNode> getParent() const;
    void changeParent(shared_ptr<SearchTreeNode> newParent, shared_ptr<SearchTreeNode>& selfPtr);

    // Get children
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> getChildren() const;

    // Get state
    SS_VECTOR* getState();

    // Cost
    float getSegmentCost(void) const;
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost) const;

    // Trace back to parent and add state
    void traceBackToRoot(PATH_TYPE* stateVector) const;

    // Set isRoot property
    void setRoot(bool isNodeRoot);

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(childNodes), cereal::defer(parentNode), state, cost, isRoot);}

};



#endif //SEARCHTREENODE_H