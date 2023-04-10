#ifndef types_h
#define types_h

#define SS_VECTOR   std::vector<double>
#define PATH_TYPE   std::vector<SS_VECTOR>

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
    float goalBias;
    float goalRadius;
    int iterations;
    float maxConeDist;
    int maxNumOfNodes;
    float sampleRange;
    float maxVelocity;
    float resolution;
    float rewireRange;
    float simulationTimeStep;
};

struct VEHICLE_PARAMETERS{
    float track;
    float wheelBase;
    SIMULATION_TYPE simType;
    DISTANCE_TYPE distType;
    COST_TYPE costType;
};

struct MAP_PARAMETERS{
    float goalHorizon;
};

struct GENERAL_PARAMETERS{
    float timerPeriod;
};

#endif