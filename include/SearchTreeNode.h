#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <ros/ros.h>
#include <iterator>
#include "Types.h"
#include "StateSpaceSimulated.h"

class SearchTreeNode
{
private:
    SearchTreeNode* parentNode;
    std::vector<SearchTreeNode *> *childNodes;
    SS_VECTOR state;
    float cost;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode &original);
    SearchTreeNode(SearchTreeNode* parent, SS_VECTOR stateSpace, double nodeCost);

    // Destructor
    ~SearchTreeNode();

    // Add child node
    void addChild(SearchTreeNode* childNode);

    // Remove child node
    void removeChild(SearchTreeNode* childNode);

    // Get and change parent
    SearchTreeNode* getParent();
    void changeParent(SearchTreeNode* newParent);

    // Get children
    std::vector<SearchTreeNode*> *getChildren();

    // Get state
    SS_VECTOR* getState();

    // Cost
    float getSegmentCost(void);
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost);

    // Trace back to parent and add state
    void traceBackToRoot(PATH_TYPE* stateVector);

};



#endif //SEARCHTREENODE_H