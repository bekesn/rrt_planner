#ifndef types_h
#define types_h

#define SS_VECTOR   std::vector<double>
#define PATH_TYPE   std::vector<SS_VECTOR>

typedef enum{
    LOCAL_RRT,
    GLOBAL_RRT
}RRT_TYPE;

typedef enum{
    HOLONOMIC,
    HOLONOMIC_CONSTRAINED,
    BICYCLE_SIMPLE,
    BICYCLE
}SIMULATION_TYPE;

typedef enum{
    EUCLIDEAN,
    SIMULATED
}DISTANCE_TYPE;

typedef enum{
    TIME,
    DISTANCE
}COST_TYPE;

typedef struct RRT_PARAMETERS{
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

typedef struct VEHICLE_PARAMETERS{
    float track;
    float wheelBase;
    SIMULATION_TYPE simType;
    DISTANCE_TYPE distType;
    COST_TYPE costType;
};

typedef struct MAP_PARAMETERS{
    float goalHorizon;
};


#endif