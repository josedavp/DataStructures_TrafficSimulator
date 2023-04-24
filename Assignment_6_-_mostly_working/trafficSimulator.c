#include "trafficSimulator.h"


/* trafficSimulator
 * input: char* name of file to read
 * output: N/A
 *
 * Simulate the road network in the given file
 */
void trafficSimulator(char* filename)
{
    int i;

    int currentTimestep = 0;
    Event* event;
    Car* car;
    int numCarsMovedThisCycle;

    //statistics
    int totalTimestepsAllCarsTook = 0;
    int maxTimestepsAnyCarTook = 0;
    int numCarsExit = 0;

    TrafficSimData* trafficSimData = (TrafficSimData*) malloc(sizeof(TrafficSimData));
    trafficSimData->eventPQ = createPQ();
    trafficSimData->numCarsInSimulator = 0;
    trafficSimData->longestLightCycle = 0;
    
    /* Read in the traffic data.  You may want to create a separate function to do this. */
    readTrafficData(trafficSimData, filename);
    
    /* Loop until all events processed and either all cars reached destination or gridlock has occurred */
    int gridlock = 0;
    bool firstTimestep = true;
    int pnext;
    RoadData *currentRoad, *nextRoad;
    int time;
    while(firstTimestep || !isEmptyPQ(trafficSimData->eventPQ) || !getCarsRemaining(trafficSimData) == 0) {
        if(firstTimestep) firstTimestep = false;
        numCarsMovedThisCycle = 0;
        
        /* Loop on events associated with this time step */
        while(!isEmptyPQ(trafficSimData->eventPQ) && getFrontPriority(trafficSimData->eventPQ) == currentTimestep) {
            event = dequeuePQ(trafficSimData->eventPQ);
            if(event->eventType == ADD_CAR_EVENT) {
                car = (Car*) malloc(sizeof(Car));
                car->destVertex = event->destVertex;
                car->timestepEntered = currentTimestep; //TODO: does timer start when cars are added to waitlist or when cars are added to road?
                enqueue(getEdgeData(trafficSimData->roadNetwork, event->fromVertex, event->toVertex)->waitlist, car);
                printf("CYCLE %d - ADD_CAR_EVENT - Car enqueued on road from %d to %d\n", currentTimestep, event->fromVertex, event->toVertex);
                trafficSimData->numCarsInSimulator++;
                numCarsMovedThisCycle++;
            } else if(event->eventType == PRINT_ROADS_EVENT) {
                printf("\nCYCLE %d - PRINT_ROADS_EVENT - Current contents of the roads:\n", currentTimestep);
                for(i=0; i < trafficSimData->numRoads; i++) {
                    currentRoad = trafficSimData->roadArray[i];
                    printf("  Road %d -> %d has %d cars on the road and %d cars waiting to enter\n", 
                        currentRoad->fromVertex, 
                        currentRoad->toVertex, 
                        currentRoad->cars->numElements, 
                        currentRoad->waitlist->numElements
                    );
                }
            }
            free(event);
        }
        
        /* First try to move through the intersection */
        for(i=0; i < trafficSimData->numRoads; i++) {
            currentRoad = trafficSimData->roadArray[i];
            //if car reached its destination, remove it
            if(!isEmpty(currentRoad->cars) && getNext(currentRoad->cars)->destVertex == currentRoad->toVertex) {
                car = dequeue(currentRoad->cars);
                time = currentTimestep - car->timestepEntered;
                totalTimestepsAllCarsTook += time;
                if(time > maxTimestepsAnyCarTook) maxTimestepsAnyCarTook = time;
                numCarsExit++;
                trafficSimData->numCarsInSimulator--;
                numCarsMovedThisCycle++;
                printf("car exited road %d at vertex %d; it's destination was vertex %d\n", i, currentRoad->toVertex, car->destVertex);
                free(car);
            //if there's a car on the current road AND there's a path to the next road AND the light is green
            //TODO: if or else-if? when does a car technically reach its destination?
            } else if(!isEmpty(currentRoad->cars)
              && getNextOnShortestPath(trafficSimData->roadNetwork, currentRoad->toVertex, getNext(currentRoad->cars)->destVertex, &pnext) //next road's toVertex is stored in pnext
              && (currentTimestep % currentRoad->cycleReset >= currentRoad->greenOn && currentTimestep % currentRoad->cycleReset < currentRoad->greenOff)) { 
                nextRoad = getEdgeData(trafficSimData->roadNetwork, currentRoad->toVertex, pnext);
                //if there's space on the next road
                if(nextRoad->cars->numElements < nextRoad->length - 1) {
                    //move the current car to the end of the next road
                    enqueue(nextRoad->cars, dequeue(currentRoad->cars));
                    numCarsMovedThisCycle++;
                    printf("car moved from %d -> %d to %d -> %d\n", 
                        currentRoad->fromVertex, currentRoad->toVertex, 
                        nextRoad->fromVertex, nextRoad->toVertex
                    );
                }
            }
        }
        
        /* Second move cars forward on every road */
        // N/A since the queue handles this
        
        /* Third move cars onto road if possible */
        for(i=0; i < trafficSimData->numRoads; i++) {
            currentRoad = trafficSimData->roadArray[i];
            if(!isEmpty(currentRoad->waitlist) && currentRoad->cars->numElements < currentRoad->length - 1) {
                enqueue(currentRoad->cars, dequeue(currentRoad->waitlist));
                numCarsMovedThisCycle++;
            }
        }

        //detect gridlock
        if(numCarsMovedThisCycle == 0) gridlock++;
        int offset = 3; //not necessary, but can't hurt
        if(gridlock > trafficSimData->longestLightCycle + offset) {
            printf("\nCYCLE %d - Gridlock has been detected.\n", currentTimestep - offset);
            break; //break from simulation loop
        }
        
        currentTimestep++;
    }

    //print statistics
    if(numCarsExit > 0) {
        printf("\nAverage number of time steps to the reach their destination is %0.1f.\n", (double) totalTimestepsAllCarsTook/numCarsExit);
        printf("Maximum number of time steps to the reach their destination is %d.\n\n", maxTimestepsAnyCarTook);
    } else {
        printf("\nNo cars made it :(\n");
        printf("WCKD wins.\n\n");
    }
    printf("%d cars remaining on the road.\n\n", trafficSimData->numCarsInSimulator);

    //cleanup
    freeGraph(trafficSimData->roadNetwork);
    for(i=0; i < trafficSimData->numRoads; i++) {
        currentRoad = trafficSimData->roadArray[i];
        freeQueue(currentRoad->cars);
        freeQueue(currentRoad->waitlist);
        free(currentRoad);
    }
    free(trafficSimData->roadArray);
    freePQ(trafficSimData->eventPQ);
    free(trafficSimData);
}

void readTrafficData(TrafficSimData* trafficSimData, char* filename) {
    printf("\nREADING DATA FILE:\n\n");
    int i, j;
    
    FILE* file = fopen(filename, "r");
    if(file == NULL) {
        printf("Error opening %s\n", filename);
        exit(-1);
    }

    int numVerticies, numEdges;

    fscanf(file, "%d %d", &numVerticies, &numEdges);
    printf("<%d> <%d>\n", numVerticies, numEdges);

    Graph* g = createGraph(numVerticies);
    trafficSimData->roadArray = (RoadData**) malloc(sizeof(RoadData*) * numEdges);
    trafficSimData->numRoads = 0;

    int fromVertex, toVertex, numIncomingRoads;
    int length, greenOn, greenOff, cycleReset;

    for(i=0; i < numVerticies; i++) {
        fscanf(file, "%d %d", &toVertex, &numIncomingRoads);
        printf("\n<%d> <%d>\n", toVertex, numIncomingRoads);
        for(j=0; j < numIncomingRoads; j++) {
            fscanf(file, "%d %d %d %d %d", &fromVertex, &length, &greenOn, &greenOff, &cycleReset);
            if(cycleReset > trafficSimData->longestLightCycle) trafficSimData->longestLightCycle = cycleReset;
            printf("<%d> <%d> <%d> <%d> <%d>\n", fromVertex, length, greenOn, greenOff, cycleReset);
            RoadData* road = (RoadData*) malloc(sizeof(RoadData));
            road->greenOn = greenOn;
            road->greenOff = greenOff;
            road->cycleReset = cycleReset;
            road->cars = createQueue();
            road->waitlist = createQueue();
            setEdge(g, fromVertex, toVertex, true); //note edge default is -1 not 0/NULL/false
                                                    //implicitly creates verticies
            setEdgeData(g, fromVertex, toVertex, road);
            setDistance(g, fromVertex, toVertex, length);
            //store redundant, but useful information
            road->length = length;
            road->fromVertex = fromVertex;
            road->toVertex = toVertex;
            trafficSimData->roadArray[trafficSimData->numRoads] = road;
            printf("Created road %d to vertex index %d from vertex #%d (number, not index) \n", trafficSimData->numRoads, i, j);
            trafficSimData->numRoads++;
        }
    }

    int numAddCarEvents;
    int timestep, numCarsToAdd, destVertex;
    Event* event;

    fscanf(file, "%d", &numAddCarEvents);
    printf("\n\n<%d>", numAddCarEvents);

    for(i=0; i < numAddCarEvents; i++) {
        fscanf(file, "%d %d %d", &fromVertex, &toVertex, &timestep);
        printf("\n\n<%d> <%d> <%d>\n", fromVertex, toVertex, timestep);
        fscanf(file, "%d", &numCarsToAdd);
        printf("<%d>\n", numCarsToAdd);
        for(j=0; j < numCarsToAdd; j++) {
            fscanf(file, "%d", &destVertex);
            printf("<%d> ", destVertex);
            event = (Event*) malloc(sizeof(Event));
            event->eventType = ADD_CAR_EVENT;
            event->fromVertex = fromVertex;
            event->toVertex = toVertex;
            event->destVertex = destVertex;
            enqueueByPriority(trafficSimData->eventPQ, event, timestep);
        }
    }

    int numPrintRoadCMDs;

    fscanf(file, "%d", &numPrintRoadCMDs);
    printf("\n\n\n<%d>\n", numPrintRoadCMDs);

    for(i=0; i < numPrintRoadCMDs; i++) {
        fscanf(file, "%d", &timestep);
        printf("<%d> ", timestep);
        Event* event = (Event*) malloc(sizeof(Event));
        event->eventType = PRINT_ROADS_EVENT;
        enqueueByPriority(trafficSimData->eventPQ, event, timestep);
    }
    printf("\n\n");
    fflush(stdout);

    fclose(file);
    printf("DONE READING DATA FILE.\n\n");
    trafficSimData->roadNetwork = g;
}

int getCarsRemaining(TrafficSimData* trafficSimData) {
    int i;
    int carsRemaining = 0;

    for(i=0; i < trafficSimData->numRoads; i++) {
        carsRemaining += trafficSimData->roadArray[i]->cars->numElements;
        carsRemaining += trafficSimData->roadArray[i]->waitlist->numElements;
    }
    return carsRemaining;
}