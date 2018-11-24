//
// Created by Andrei Neagu on 17/11/18.
//

#ifndef PATH_PLANNING_STATEMACHINE_H
#define PATH_PLANNING_STATEMACHINE_H

#include "Vehicles.h"

#include <map>

using namespace std;


enum CarStates {
    KEEP_LANE,
    CHANGE_LANE_RIGHT,
    CHANGE_LANE_LEFT
};


/*
 * Simplified way to handle input in a state
 * */
class StateInput {
public:
    VehicleTracker *vehicleTracker;
    int carLane;
    double carS;
    double carD;
    double carSpeed;
    double laneWidth;

    StateInput(VehicleTracker *vehicleTracker, int carLane, double carS, double carD, double carSpeed,
               double laneWidth) {
        this->vehicleTracker = vehicleTracker;
        this->carLane = carLane;
        this->carS = carS;
        this->carD = carD;
        this->carSpeed = carSpeed;
        this->laneWidth = laneWidth;
    }

};

/*
 * Simplified way to handle output form a state
 * */
class StateOutput {
public:
    int nextLane;
    int extraLaneToMonitor;
    CarStates newState;

    StateOutput() = default;
};

class StateMachine {
private:
    typedef StateOutput (*class_method)(StateInput stateInput);

    map<CarStates, class_method> statesHandler;

    /*
     * Stays in the same lane
     * When it is safe to change a lane it will initialize a lane change
     * */
    static StateOutput handleStateKeepLane(StateInput stateInput);

    /*
     * Responsible for moving to the lane to the right
     * */
    static StateOutput handleStateChangeLaneRight(StateInput stateInput);

    /*
    * Responsible for moving to the lane to the left
    * */
    static StateOutput handleStateChangeLaneLeft(StateInput stateInput);

    CarStates currentState;

    static string getStringCarStates(CarStates carState) {
        switch (carState) {
            case KEEP_LANE:
                return "keep_lane";
            case CHANGE_LANE_RIGHT:
                return "change_right";
            case CHANGE_LANE_LEFT:
                return "change_left";
        }
        return "";
    }


public:
    StateMachine() {
        currentState = KEEP_LANE;

        statesHandler[KEEP_LANE] = handleStateKeepLane;
        statesHandler[CHANGE_LANE_RIGHT] = handleStateChangeLaneRight;
        statesHandler[CHANGE_LANE_LEFT] = handleStateChangeLaneLeft;
    }

    StateOutput runOnce(StateInput &stateInput) {
        StateOutput stateOutput = statesHandler[currentState](stateInput);
        currentState = stateOutput.newState;
        //cout << "STATE " << getStringCarStates(currentState) << endl;
        return stateOutput;
    }

};


#endif //PATH_PLANNING_STATEMACHINE_H
