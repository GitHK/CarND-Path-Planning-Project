//
// Created by Andrei Neagu on 11/11/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <math.h>
#include "string.h"
#include <chrono>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

enum LaneToGoTo {
    LEFT, RIGHT, MINE
};

using namespace std;

double deg2rad(double x);

double rad2deg(double x);

string hasData(string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

double mphToMs(double speed_mph);

double msToMph(double speed_ms);

bool inRange(double low, double high, double x);

void printVector(vector<double> path);

LaneToGoTo getLaneToGoTo(int carLane, int laneToDriveIn);

#endif //PATH_PLANNING_UTILS_H
