#include <SearchTreeNode.h>

SearchTreeNode::SearchTreeNode()
{
    childNodes = new std::vector<SearchTreeNode*>(0);
    isRoot = false;
}


SearchTreeNode::SearchTreeNode(const SearchTreeNode &original)
{
    parentNode = original.parentNode;
    //childNodes = original.childNodes;
    childNodes = new std::vector<SearchTreeNode*>;
    copy(original.childNodes->begin(), original.childNodes->end(), back_inserter(*childNodes));
    state = original.state;
    cost = original.cost;
    isRoot = original.isRoot;
}

SearchTreeNode::SearchTreeNode(SearchTreeNode* parent, SS_VECTOR stateSpace, double nodeCost)
{
    parentNode = parent;
    state = stateSpace;
    childNodes = new std::vector<SearchTreeNode*>(0);
    cost = nodeCost;
    isRoot = false;
}

SearchTreeNode::~SearchTreeNode()
{
    std::vector<SearchTreeNode*>::iterator it;
    for (it = childNodes->begin(); it != childNodes->end(); it++)
    {
        delete *it;
    }
    delete childNodes;
}


void SearchTreeNode::addChild(SearchTreeNode* childNode)
{
    childNodes->push_back(childNode);
}

void SearchTreeNode::removeChild(SearchTreeNode* childNode)
{
    childNodes->erase(std::remove(childNodes->begin(), childNodes->end(), childNode),childNodes->end());
}

SearchTreeNode* SearchTreeNode::getParent()
{
    return parentNode;
}

void SearchTreeNode::changeParent(SearchTreeNode* newParent)
{
    parentNode = newParent;
}

std::vector<SearchTreeNode*> *SearchTreeNode::getChildren()
{
    return childNodes;
}

SS_VECTOR* SearchTreeNode::getState()
{
    return &state;
}

float SearchTreeNode::getSegmentCost(void)
{
    return cost;
}

void SearchTreeNode::changeSegmentCost(float newCostValue)
{
    cost = newCostValue;
}

void SearchTreeNode::addToAbsoluteCost(float* absCost)
{
    *absCost += cost;
    if (parentNode != NULL) parentNode->addToAbsoluteCost(absCost);
}

void SearchTreeNode::traceBackToRoot(PATH_TYPE* stateVector)
{
    // TODO: inserting at the beginning might slow down the process
    stateVector->insert(stateVector->begin(), state);
    if ((parentNode != NULL) && !isRoot) parentNode->traceBackToRoot(stateVector);
}

