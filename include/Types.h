#ifndef types_h
#define types_h

enum RRT_TYPE{
    LOCAL_RRT,
    GLOBAL_RRT
};

enum SIMULATION_TYPE{
    HOLONOMIC,
    KINEMATIC,
    DYNAMIC
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
    float collisionRadius;
    float goalHorizon;
    float maxConeDist;
    float maxGap;
};

struct CONTROL_PARAMETERS{
    float kPsi;
    float kDelta;
};

struct GENERAL_PARAMETERS{
    float timerPeriod;
};

#endif