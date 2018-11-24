//
// Created by Andrei Neagu on 11/11/18.
//

#ifndef PATH_PLANNING_VEHICLES_H
#define PATH_PLANNING_VEHICLES_H

#include <string>
#include <map>
#include <math.h>
#include <vector>
#include "json.hpp"

using json = nlohmann::json;

using namespace std;


class VehicleData {
private:
    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;

public:
    double id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
    double acceleration;

    VehicleData();

    VehicleData(double id, vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    void update(double x, double y, double vx, double vy, double s, double d);

    void deltaElapsed(double deltaTime);

    friend bool operator<(VehicleData const &a, VehicleData const &b) {
        return a.s < b.s;
    }

    static void printVehiclesS(vector<VehicleData> vehicles) {
        cout << "[";
        for (auto &vehicle : vehicles)
            cout << vehicle.s << ", ";
        cout << "]" << endl;
    }

};

class VehicleTracker {

private:
    map<double, VehicleData> trackedVehicles;
    map<int, vector<VehicleData>> laneToCarsMapping;

    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;

    int lastConvenientLane = 1; // starting lane
    int newConvenientLaneCounter = 0;

    int getMostConvenientLaneToUse(int carLane, double carS);

    double getCost(int carLane, int otherVehicleLane, double carS, vector<VehicleData> vehicles);

    bool isLaneClearForLaneChange(vector<VehicleData> vehiclesBehindTheCar, vector<VehicleData> vehiclesAheadOfCar,
                                  double carS, double carSpeed);

public:
    VehicleTracker(vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    VehicleTracker() = default;

    void updateFromJsonOrMoveTimeForNotFoundVehicles(json sensorFusionCarsArray);

    void sortVehiclesByLane(double laneWidth);

    vector<VehicleData> getVehiclesInRangeOfCar(int lane, double carS, double backDelta, double frontDelta);

    vector<VehicleData> getVehiclesInFrontOfCarInvadingAndExtraMonitoredLanes(int lane, double carS,
                                                                              double frontDelta,
                                                                              double percentLateralDelta,
                                                                              double laneWidth,
                                                                              int extraLaneToMonitor);

    int getLaneToDriveIn(int carLane, double carS);

    bool isRightLaneClearForLaneChange(int carLane, double carS, double carSpeed);

    bool isLeftLaneClearForLaneChange(int carLane, double carS, double carSpeed);

    bool isFirstVehicleBehindSlowerThenCar(int carLane, double carS, double carSpeed);

};


#endif //PATH_PLANNING_VEHICLES_H
