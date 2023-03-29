#ifndef types_h
#define types_h

#define SS_VECTOR   std::vector<double>
#define PATH_TYPE   std::vector<SS_VECTOR>

typedef struct RRT_PARAMETERS{
    float collisionRange;
    float goalBias;
    float goalRadius;
    float goalHorizon;
    float maxConeDist;
    float maxNumOfNodes;
    float sampleRange;
    float maxVelocity;
    float resolution;
    float rewireRange;
    float simulationTimeStep;
    float track;
    float wheelBase;
};




#endif