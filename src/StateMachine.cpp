//
// Created by Andrei Neagu on 17/11/18.
//

#include "StateMachine.h"
#include "utils.h"

#define LINE_CENTER_PERCENTAGE_OF_TOTAL_LANE 0.1

//TODO: forward lateral checkin may be borken?
//TODO: It could be lateral checking forward during overtakeing for fron object if too close, it will not avoid it, must match its speed
// ONLY during lane changes the we are not correctly monitoring lanes, the current lane and the old lane!

StateOutput StateMachine::handleStateKeepLane(StateInput stateInput) {
    int carLane = stateInput.carLane;
    double carS = stateInput.carS;
    double carSpeed = stateInput.carSpeed;

    StateOutput stateOutput = StateOutput();

    int targetLane = stateInput.vehicleTracker->getLaneToDriveIn(carLane, carS);
    stateOutput.nextLane = carLane;
    stateOutput.newState = KEEP_LANE;

    LaneToGoTo laneToGoTo = getLaneToGoTo(carLane, targetLane);
    //cout << "GOTO " << laneToGoTo << "car lane " << carLane << " target lane " << targetLane << endl;
    if (laneToGoTo == RIGHT && stateInput.vehicleTracker->isRightLaneClearForLaneChange(carLane, carS, carSpeed)) {
        stateOutput.nextLane = carLane + 1;
        stateOutput.newState = CHANGE_LANE_RIGHT;
    } else if (laneToGoTo == LEFT && stateInput.vehicleTracker->isLeftLaneClearForLaneChange(carLane, carS, carSpeed)) {
        stateOutput.nextLane = carLane - 1;
        stateOutput.newState = CHANGE_LANE_LEFT;
    }

    // actually we monitor the same lane so its ok
    stateOutput.extraLaneToMonitor = stateOutput.nextLane;

    return stateOutput;
}

StateOutput StateMachine::handleStateChangeLaneRight(StateInput stateInput) {
    double laneWidth = stateInput.laneWidth;
    double carD = stateInput.carD;
    int carLane = stateInput.carLane;

    StateOutput stateOutput = StateOutput();
    stateOutput.newState = CHANGE_LANE_RIGHT;
    stateOutput.nextLane = carLane;
    int originalLane = stateOutput.nextLane - 1;


    double laneCenter = carLane * laneWidth + laneWidth / 2;
    double laneCenterLowerBound = laneCenter - (laneCenter * LINE_CENTER_PERCENTAGE_OF_TOTAL_LANE / 2);
    double laneCenterUpperBound = laneCenter + (laneCenter * LINE_CENTER_PERCENTAGE_OF_TOTAL_LANE / 2);

    //if car behind is faster ABORT and go back to the original lane
    if (!stateInput.vehicleTracker->isFirstVehicleBehindSlowerThenCar(carLane, stateInput.carS, stateInput.carSpeed)) {
        stateOutput.nextLane = originalLane;
        stateOutput.newState = KEEP_LANE;
        cout << "[CLR] ABORTING, going back to lane: " << stateOutput.nextLane << " from " << carLane << endl;
        return stateOutput;
    }

    cout << "[CLR]"
         << "L " << carLane << " "
         << "< " << laneCenterLowerBound << " "
         << "| " << laneCenter << " "
         << "> " << laneCenterUpperBound << " "
         << endl;

    // check if car is in desired line, it is now safe to change line, wh check in the center 10% of the lane
    if (carD > laneCenterLowerBound && carD < laneCenterUpperBound)
        stateOutput.newState = KEEP_LANE;

    stateOutput.extraLaneToMonitor = originalLane;
    return stateOutput;
}

StateOutput StateMachine::handleStateChangeLaneLeft(StateInput stateInput) {
    double laneWidth = stateInput.laneWidth;
    double carD = stateInput.carD;
    int carLane = stateInput.carLane;

    StateOutput stateOutput = StateOutput();
    stateOutput.newState = CHANGE_LANE_LEFT;
    stateOutput.nextLane = carLane;
    int originalLane = stateOutput.nextLane + 1;

    double laneCenter = carLane * laneWidth + laneWidth / 2;
    double laneCenterLowerBound = laneCenter - (laneCenter * LINE_CENTER_PERCENTAGE_OF_TOTAL_LANE / 2);
    double laneCenterUpperBound = laneCenter + (laneCenter * LINE_CENTER_PERCENTAGE_OF_TOTAL_LANE / 2);

    //if car behind is faster ABORT and go back to the original lane
    if (!stateInput.vehicleTracker->isFirstVehicleBehindSlowerThenCar(carLane, stateInput.carS, stateInput.carSpeed)) {
        stateOutput.nextLane = originalLane;  // go back to original lane
        stateOutput.newState = KEEP_LANE;
        cout << "[CLL] ABORTING, going back to lane: " << stateOutput.nextLane << " from " << carLane << endl;
        return stateOutput;
    }

    cout << "[CLL]"
         << "L " << carLane << " "
         << "< " << laneCenterLowerBound << " "
         << "| " << laneCenter << " "
         << "> " << laneCenterUpperBound << " "
         << endl;

    // check if car is in desired line, it is now safe to change line, wh check in the center 10% of the lane
    if (carD > laneCenterLowerBound && carD < laneCenterUpperBound)
        stateOutput.newState = KEEP_LANE;

    stateOutput.extraLaneToMonitor = originalLane;
    return stateOutput;
}

