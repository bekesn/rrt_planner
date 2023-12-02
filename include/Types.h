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
    float resolution;
    float rewireTime;
    int simIterations;
    float simulationTimeStep;
    float thetaWeight;
    float triangleIterations;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(costType, goalBias, goalRadius,
                    iterations, maxNumOfNodes, minCost, minDeviation, sampleRange, 
                    resolution, rewireTime, simulationTimeStep, thetaWeight);}
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

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(simType, maxDelta, maxdDelta, maxLongAccel, maxVelocity, track, wheelBase);}
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

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(k);}
};

struct GENERAL_PARAMETERS{
    float timerPeriod;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(timerPeriod);}
};

#endif