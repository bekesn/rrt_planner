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
    float sampleRange;
    float maxVelocity;
    float resolution;
    float rewireTime;
    int simIterations;
    float simulationTimeStep;
    float thetaWeight;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(costType, goalBias, goalRadius,
                    iterations, maxNumOfNodes, minCost, minDeviation, sampleRange, 
                    maxVelocity, resolution, rewireTime, simulationTimeStep, thetaWeight);}
};

struct VEHICLE_PARAMETERS{
    SIMULATION_TYPE simType;
    float maxDelta;
    float track;
    float wheelBase;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(simType, maxDelta, track, wheelBase);}
};

struct MAP_PARAMETERS{
    float collisionRange;
    float goalHorizon;
    float maxConeDist;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(collisionRange, goalHorizon, maxConeDist);}
};

struct CONTROL_PARAMETERS{
    float k;
    float maxdDelta;
    float maxLongAccel;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(k, maxdDelta, maxLongAccel);}
};

struct GENERAL_PARAMETERS{
    float timerPeriod;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(timerPeriod);}
};

#endif