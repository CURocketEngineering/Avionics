#ifndef STATE_ESTIMATION_TYPES_H
#define STATE_ESTIMATION_TYPES_H

#include "data_handling/DataPoint.h"

struct AccelerationTriplet {
    DataPoint x;
    DataPoint y;
    DataPoint z;
};

#endif // STATE_ESTIMATION_TYPES_H