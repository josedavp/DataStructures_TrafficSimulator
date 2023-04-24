#ifndef _road_h
#define _road_h
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "car.h"
#include "queue.h"

typedef struct RoadData
{
    /* TODO - Add data associated with road.  Suggested data: */
    /* length of this road */
    int length;
    /* information used to record/update whether the light at the end of this road is green or red */
    int greenOn;
    int greenOff;
    int cycleReset;
    /* intersections this road starts from and moves to */
    int fromVertex;
    int toVertex;
    /* array of cars associated with this road */
    Queue* cars;
    /* queue of cars waiting to enter this road */
    Queue* waitlist;
}  RoadData;

#endif

