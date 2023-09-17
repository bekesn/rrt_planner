#include <SearchTreeNode.h>

SearchTreeNode::SearchTreeNode()
{
    parentNode = shared_ptr<SearchTreeNode> (new SearchTreeNode);
    cost = 0;
    isRoot = false;
    childNodes = shared_ptr<vector<shared_ptr<SearchTreeNode>>> (new vector<shared_ptr<SearchTreeNode>>);
}


SearchTreeNode::SearchTreeNode(const SearchTreeNode &original)
{
    parentNode = original.parentNode;
    childNodes = original.childNodes;
    //copy(original.childNodes->begin(), original.childNodes->end(), back_inserter(*childNodes));
    state = original.state;
    cost = original.cost;
    isRoot = original.isRoot;
}

SearchTreeNode::SearchTreeNode(shared_ptr<SearchTreeNode> parent, SS_VECTOR stateSpace, double nodeCost)
{
    parentNode = parent;
    state = stateSpace;
    cost = nodeCost;
    isRoot = false;
    childNodes = shared_ptr<vector<shared_ptr<SearchTreeNode>>> (new vector<shared_ptr<SearchTreeNode>>);
}

SearchTreeNode::~SearchTreeNode()
{

}


void SearchTreeNode::addChild(shared_ptr<SearchTreeNode> childNode)
{
    childNodes->push_back(childNode);
}

void SearchTreeNode::removeChild(shared_ptr<SearchTreeNode> childNode)
{
    childNodes->erase(std::remove(childNodes->begin(), childNodes->end(), childNode),childNodes->end());
}

shared_ptr<SearchTreeNode> SearchTreeNode::getParent() const
{
    return parentNode;
}

void SearchTreeNode::changeParent(shared_ptr<SearchTreeNode> newParent, shared_ptr<SearchTreeNode>& selfPtr)
{
    parentNode = newParent;
    newParent->addChild(selfPtr);
}

shared_ptr<vector<shared_ptr<SearchTreeNode>>> SearchTreeNode::getChildren() const
{
    return childNodes;
}

SS_VECTOR* SearchTreeNode::getState()
{
    return &state;
}

float SearchTreeNode::getSegmentCost(void) const
{
    return cost;
}

void SearchTreeNode::changeSegmentCost(float newCostValue)
{
    cost = newCostValue;
}

void SearchTreeNode::addToAbsoluteCost(float* absCost) const
{
    *absCost += cost;
    if ((parentNode != NULL) && !isRoot) parentNode->addToAbsoluteCost(absCost);
}

void SearchTreeNode::traceBackToRoot(shared_ptr<PATH_TYPE>& stateVector) const
{
    // TODO: inserting at the beginning might slow down the process
    stateVector->insert(stateVector->begin(), state);
    if ((parentNode != NULL) && !isRoot) parentNode->traceBackToRoot(stateVector);
}

void SearchTreeNode::setRoot(bool isNodeRoot)
{
    isRoot = isNodeRoot;
}