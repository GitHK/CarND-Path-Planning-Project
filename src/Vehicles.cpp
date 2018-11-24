//
// Created by Andrei Neagu on 11/11/18.
//

#include "Vehicles.h"
#include "utils.h"

VehicleData::VehicleData() = default;

VehicleData::VehicleData(double id, vector<double> &maps_s, const vector<double> &maps_x,
                         const vector<double> &maps_y) {
    this->id = id;
    this->maps_s = maps_s;
    this->maps_x = maps_x;
    this->maps_y = maps_y;
}

// update with new data from sensor fusion
void VehicleData::update(double newX, double newY, double newVx, double newVy, double newS, double newD) {
    x = newX;
    y = newY;
    vx = newVx;
    vy = newVy;
    s = newS;
    d = newD;

    // compute speed
    double newSpeed = sqrt(vx * vx + vy * vy);
    acceleration = (newSpeed - speed) / 0.02;
    speed = newSpeed;
}

// changes the position of the car, this must be used if no data was provided by update function
void VehicleData::deltaElapsed(double deltaTime) {
    // move the car in the s direction by speed and then
    double deltaDistance = deltaTime * speed;
    s += deltaDistance;
    acceleration = 0;   // no change in speed means acceleration is 0

    vector<double> xy = getXY(s, d, maps_s, maps_x, maps_y);
    x = xy[0];
    y = xy[1];
}


VehicleTracker::VehicleTracker(vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    this->maps_s = maps_s;
    this->maps_x = maps_x;
    this->maps_y = maps_y;
}


void VehicleTracker::updateFromJsonOrMoveTimeForNotFoundVehicles(json sensorFusionCarsArray) {
    map<double, int> updatedVehiclesId;

    for (auto &element: sensorFusionCarsArray) {
        //cout << element << endl;
        double id = element[0];
        double x = element[1];
        double y = element[2];
        double vx = element[3];
        double vy = element[4];
        double s = element[5];
        double d = element[6];

        // keep track of elements for a fast search
        updatedVehiclesId[id] = 0;

        auto iterator = trackedVehicles.find(id);

        if (iterator != trackedVehicles.end()) {
            //element update existing data
            //cout << "will update" << endl;
            iterator->second.update(x, y, vx, vy, s, d);
        } else {
            // create a new element and add it
            //cout << "will add" << endl;
            VehicleData vehicleData = VehicleData(id, maps_s, maps_x, maps_y);
            vehicleData.update(x, y, vx, vy, s, d);
            trackedVehicles[id] = vehicleData;
        }
    }
}

void VehicleTracker::sortVehiclesByLane(double laneWidth) {

    for (auto &iterator : laneToCarsMapping) {
        int lane = iterator.first;
        laneToCarsMapping[lane].clear();
    }

    // store vehicles by lane
    for (auto const &trackedVehicle : trackedVehicles) {
        VehicleData vehicle = trackedVehicle.second;

        if (vehicle.d >= (laneWidth * (0 + 0)) && vehicle.d < (laneWidth * (1 + 0)))
            laneToCarsMapping[0].push_back(vehicle);

        if (vehicle.d >= (laneWidth * (0 + 1)) && vehicle.d <= (laneWidth * (1 + 1)))
            laneToCarsMapping[1].push_back(vehicle);

        if (vehicle.d > (laneWidth * (0 + 2)) && vehicle.d <= (laneWidth * (1 + 2)))
            laneToCarsMapping[2].push_back(vehicle);
    }

    // sort vehicles in lanes
    for (auto &iterator : laneToCarsMapping) {
        int lane = iterator.first;
        std::sort(laneToCarsMapping[lane].begin(), laneToCarsMapping[lane].end());
    }

}


// returns a list of cars in the specified range with respect to the current car's position
vector<VehicleData>
VehicleTracker::getVehiclesInRangeOfCar(int lane, double carS, double backDelta, double frontDelta) {
    vector<VehicleData> foundVehicles;

    double lowBound = carS - backDelta;
    double highBound = carS + frontDelta;

    for (auto &vehicleData : laneToCarsMapping[lane]) {
        if (vehicleData.s > highBound)
            break;

        if (lowBound < vehicleData.s)
            foundVehicles.push_back(vehicleData);
    }

    return foundVehicles;
}


vector<VehicleData> VehicleTracker::getVehiclesInFrontOfCarInvadingAndExtraMonitoredLanes(int lane,
                                                                                          double carS,
                                                                                          double frontDelta,
                                                                                          double percentLateralDelta,
                                                                                          double laneWidth,
                                                                                          int extraLaneToMonitor) {
    vector<VehicleData> foundVehicles;

    // FRONT SEARCH RANGE [low = car positon, high = distance from car position moving ahead]
    double lowBound12 = carS - 0.;    // backDelta =0
    double highBound12 = carS + frontDelta;

    /* 1. CURRENT LANE MONITOR */

    // obstacles in current lane [vehicles in the lane are sorted]
    for (auto &vehicleData : laneToCarsMapping[lane]) {
        if (vehicleData.s > highBound12)
            break;

        if (lowBound12 < vehicleData.s)
            // the vehicle is in range
            foundVehicles.push_back(vehicleData);
    }

    /* 2. LATERAL CHECK */

    // compute lateralDelta used to check for invading cars
    double lateralDelta = laneWidth * percentLateralDelta;

    // obstacles in the RIGHT lane (check if there is a RIGHT lane) [vehicles in the lane are sorted]
    if (lane < 2) {
        // (lane+1) == RIGHT LANE LINE
        double rightCheckMargin = (lane + 1) * laneWidth + lateralDelta;
        for (auto &vehicleData : laneToCarsMapping[lane + 1]) {
            if (vehicleData.s > highBound12)
                break;

            if (lowBound12 < vehicleData.s)
                // the vehicle is in range
                if (vehicleData.d < rightCheckMargin) {
                    foundVehicles.push_back(vehicleData);
                    cout << ">>> detected invading car RIGHT" << endl;
                }
        }
    }

    // obstacles in the LEFT lane (check if there is a LEFT lane) [vehicles in the lane are sorted]
    if (lane > 0) {
        // lane == LEFT LANE LINE
        double leftCheckMargin = lane * laneWidth - lateralDelta;
        for (auto &vehicleData : laneToCarsMapping[lane - 1]) {
            if (vehicleData.s > highBound12)
                break;

            if (lowBound12 < vehicleData.s)
                // the vehicle is in range
                if (vehicleData.d > leftCheckMargin) {
                    foundVehicles.push_back(vehicleData);
                    cout << "<<< detected invading car LEFT" << endl;
                }
        }
    }



    /* 3. EXTRA LANE TO MONITOR */
    double lowBound3 = carS - 0.;
    double highBound3 = carS + 5;   // saefty margin is 5 meters in front of car when leaving lane during and overtake

    // obstacles in extra monitored lane
    for (auto &vehicleData : laneToCarsMapping[extraLaneToMonitor]) {
        if (vehicleData.s > highBound3)
            break;

        if (lowBound3 < vehicleData.s)
            // the vehicle is in range
            foundVehicles.push_back(vehicleData);
    }


    return foundVehicles;
}


#define LANE_CHANGE_COST 1

// computes the cost of being in each lane. The lower the cost the better it is to be in that lane
double VehicleTracker::getCost(int carLane, int otherVehicleLane, double carS, vector<VehicleData> vehicles) {
    // just consider distance for now!
    double cost = 0;
    for (auto &vehicle : vehicles) {
        double gap = vehicle.s - carS;
        cost += gap;
    }

    int totalLaneChangeCost = (LANE_CHANGE_COST * abs(carLane - otherVehicleLane));
    if (totalLaneChangeCost != 0)
        cost += totalLaneChangeCost;

    return cost;
}


// compute the best lane in which to be at the moment
int VehicleTracker::getMostConvenientLaneToUse(int carLane, double carS) {
    double minCost = 100000.;  // initialize to a high number
    int convenientLane = 100; // initialize to a high number (100 lane street is a big number)

    for (int lane = 0; lane < 3; lane++) {
        auto vehiclesInLane = getVehiclesInRangeOfCar(lane, carS, 0, 50);
        double cost = getCost(carLane, lane, carS, vehiclesInLane);
        if (cost < minCost) {
            minCost = cost;
            convenientLane = lane;
        }
    }

    return convenientLane;
}

// Will return the lane to which the vehicle should go as soon as it is safe
int VehicleTracker::getLaneToDriveIn(int carLane, double carS) {
    int newConvenientLane = getMostConvenientLaneToUse(carLane, carS);

    if (newConvenientLane != lastConvenientLane)
        newConvenientLaneCounter++;
    else
        newConvenientLaneCounter = 0;

    if (newConvenientLaneCounter > 20) {
        lastConvenientLane = newConvenientLane;
        newConvenientLaneCounter = 0; // reset counter to ensure lane is kept for at least 100 iterations
    }

    //cout << "LC " << lastConvenientLane << "; NC " << newConvenientLane << "->NCC " << newConvenientLaneCounter << endl;

    return lastConvenientLane;
}


#define OVERTAKE_FRONT_CLEARANCE 20
#define OVERTAKE_BACK_CLEARANCE 5
#define OVERTAKE_BACK_VEHICLE_MONITOR_DISTANCE 30

bool VehicleTracker::isRightLaneClearForLaneChange(int carLane, double carS, double carSpeed) {
    if (carLane == 2)
        return false;   // cannot overtake on the right from the third lane

    vector<VehicleData> vehiclesAheadOfCar = getVehiclesInRangeOfCar(carLane + 1, carS, 0., OVERTAKE_FRONT_CLEARANCE);
    vector<VehicleData> vehiclesBehindTheCar = getVehiclesInRangeOfCar(carLane + 1, carS,
                                                                       OVERTAKE_BACK_VEHICLE_MONITOR_DISTANCE, 0.);

    return isLaneClearForLaneChange(vehiclesBehindTheCar, vehiclesAheadOfCar, carS, carSpeed);
}


bool VehicleTracker::isLeftLaneClearForLaneChange(int carLane, double carS, double carSpeed) {
    if (carLane == 0)
        return false;   // cannot overtake on the left from the first lane

    auto vehiclesAheadOfCar = getVehiclesInRangeOfCar(carLane - 1, carS, 0., OVERTAKE_FRONT_CLEARANCE);
    auto vehiclesBehindTheCar = getVehiclesInRangeOfCar(carLane - 1, carS, OVERTAKE_BACK_VEHICLE_MONITOR_DISTANCE, 0.);

    return isLaneClearForLaneChange(vehiclesBehindTheCar, vehiclesAheadOfCar, carS, carSpeed);
}

bool VehicleTracker::isLaneClearForLaneChange(vector<VehicleData> vehiclesBehindTheCar,
                                              vector<VehicleData> vehiclesAheadOfCar, double carS, double carSpeed) {
    if (!vehiclesBehindTheCar.empty()) {
        VehicleData closestVehicleBehind = vehiclesBehindTheCar[vehiclesBehindTheCar.size() - 1];
        if (closestVehicleBehind.speed > carSpeed ||
            closestVehicleBehind.s > carS - OVERTAKE_BACK_CLEARANCE)
            return false;   // car is too fast or too close
    }

    return vehiclesAheadOfCar.empty();
}

bool VehicleTracker::isFirstVehicleBehindSlowerThenCar(int carLane, double carS, double carSpeed) {
    auto vehiclesBehindTheCar = getVehiclesInRangeOfCar(carLane, carS, OVERTAKE_BACK_VEHICLE_MONITOR_DISTANCE, 0.);
    if (!vehiclesBehindTheCar.empty()) {
        VehicleData closestVehicleBehind = vehiclesBehindTheCar[vehiclesBehindTheCar.size() - 1];
        if (closestVehicleBehind.speed > carSpeed ||
            closestVehicleBehind.s > carS - OVERTAKE_BACK_CLEARANCE)
            return false;   // car is too fast or too close
    }
    return true;
}
