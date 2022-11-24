#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <ros/ros.h>
#include <iterator>

class SearchTreeNode
{
private:
    SearchTreeNode* parentNode;
    std::vector<SearchTreeNode *> *childNodes;
    std::vector<double> state;
    float cost;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode &original);
    SearchTreeNode(SearchTreeNode* parent, std::vector<double> stateSpace, double nodeCost);

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
    std::vector<double> getState();

    // Cost
    float getSegmentCost(void);
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost);

    // Trace back to parent and add state
    void traceBackToRoot(std::vector<std::vector<double>>* stateVector);

};



#endif //SEARCHTREENODE_H