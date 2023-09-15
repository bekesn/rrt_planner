#ifndef types_h
#define types_h

#define SS_VECTOR   StateSpaceSimulated
#define PATH_TYPE   Trajectory//std::vector<SS_VECTOR>

enum RRT_TYPE{
    LOCAL_RRT,
    GLOBAL_RRT
};

enum SIMULATION_TYPE{
    HOLONOMIC,
    HOLONOMIC_CONSTRAINED,
    BICYCLE_SIMPLE,
    BICYCLE
};

enum DISTANCE_TYPE{
    EUCLIDEAN,
    SIMULATED
};

enum COST_TYPE{
    TIME,
    DISTANCE
};

struct RRT_PARAMETERS{
    float collisionRange;
    COST_TYPE costType;
    float goalBias;
    float goalRadius;
    int iterations;
    float maxConeDist;
    int maxNumOfNodes;
    float minCost;
    float minDeviation;
    float sampleRange;
    float maxVelocity;
    float resolution;
    float rewireRange;
    float simulationTimeStep;
    float thetaWeight;
};

struct VEHICLE_PARAMETERS{
    SIMULATION_TYPE simType;
    float maxDelta;
    float track;
    float wheelBase;
};

struct MAP_PARAMETERS{
    float goalHorizon;
};

struct CONTROL_PARAMETERS{
    float k;
    float maxdDelta;
    float maxLongAccel;
};

struct GENERAL_PARAMETERS{
    float timerPeriod;
};

#endif