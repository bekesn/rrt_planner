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

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(collisionRange, costType, goalBias, goalRadius,
                    iterations, maxConeDist, maxNumOfNodes, minCost, minDeviation, sampleRange, 
                    maxVelocity, resolution, rewireRange, simulationTimeStep, thetaWeight);}
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
    float goalHorizon;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(goalHorizon);}
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