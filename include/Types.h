#ifndef types_h
#define types_h

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


enum PlannerState{
    NOMAP,
    LOCALPLANNING,
    WAITFORGLOBAL,
    GLOBALPLANNING
};

enum MapHandlerState{
    EMPTY,
    NOBOUNDARIES,
    BOUNDARIESGIVEN
};

struct RRT_PARAMETERS{
    COST_TYPE costType;
    float goalBias;
    float goalRadius;
    int iterations;
    int maxNumOfNodes;
    float minCost;
    float minDeviation;
    float minDeviationDist;
    float psiWeight;
    float sampleRange;
    float resolution;
    float rewireTime;
    int simIterations;
    float simulationTimeStep;
    float thetaWeight;
    float triangleIterations;
};

struct VEHICLE_PARAMETERS{
    SIMULATION_TYPE simType;
    float maxdDelta;
    float maxDelta;
    float maxLatAccel;
    float maxLongAccel;
    float maxVelocity;
    float track;
    float wheelBase;
};

struct MAP_PARAMETERS{
    float collisionRange;
    float goalHorizon;
    float maxConeDist;
    float maxGap;
};

struct CONTROL_PARAMETERS{
    float k;
};

struct GENERAL_PARAMETERS{
    float timerPeriod;
};

#endif