#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "Vehicles.h"
#include "StateMachine.h"
#include "utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;


#define PATH_MAX_POINTS 20
#define LANE_WIDTH 4.0 // meters
#define MAX_DELTA_A 8 // m/s2
#define MAX_DELTA_V MAX_DELTA_A * .02 // m/s2
#define MAX_DELTA_J 10 // m/s3
#define TARGET_VELOCITY 49.5 // mph
#define SAFE_DISTANCE_IN_FRONT 30 // m

#define TRAJECTORY_POINT_COUNT 30
#define TRAJECTORY_SPACING 3   // m

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // some variables initialized here

    VehicleTracker vehicleTracker = VehicleTracker(map_waypoints_s, map_waypoints_x, map_waypoints_y);

    int lane = 1;   // car start in lane 1, possible lanes [0, 1, 2]
    int extraLaneToMonitor = lane;

    double ref_vel = 0.0;   // mph
    double target_vel = TARGET_VELOCITY; // mph

    StateMachine stateMachine = StateMachine();


    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane,
                        &ref_vel, &target_vel, &vehicleTracker, &stateMachine, &extraLaneToMonitor](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    // keep track of vehicles detected by sensor fusion or update
                    vehicleTracker.updateFromJsonOrMoveTimeForNotFoundVehicles(sensor_fusion);

                    int prev_size = previous_path_x.size();

                    //cout << "have data" << sensor_fusion.size() << endl;

                    bool leading_vehicle_detected = false;
                    double leading_vehicle_speed_mph = 1000000.; // Initialize to an impossible speed

                    // stores the vehicles in lanes in sorted by frenet s coordinate
                    vehicleTracker.sortVehiclesByLane(LANE_WIDTH);


                    // HANDLE VEHICLE IN FRONT & INVADING VEHICLES FROM SIDE LANES & EXTRA LANE TO MONITOR
                    auto obstacles = vehicleTracker.getVehiclesInFrontOfCarInvadingAndExtraMonitoredLanes(
                            lane, car_s, 400, 0.3, LANE_WIDTH, extraLaneToMonitor);
                    if (!obstacles.empty()) {
                        // check for the slowest vehicle

                        for (auto vehicleData : obstacles) {
                            // if vehicle is in front check for the gap
                            double gap = vehicleData.s - car_s;
                            if (gap > 0 && gap < SAFE_DISTANCE_IN_FRONT) {
                                //cout << "gap " << vehicle.id << " " << gap << " " << vehicle.d << endl;
                                leading_vehicle_detected = true;
                                // limiting Velocity keeps a max distance from car easier
                                double current_leading_vehicle_speed_mph = msToMph(vehicleData.speed) * 0.95;

                                // keep the car which the slowest speed in this gap range
                                if (current_leading_vehicle_speed_mph < leading_vehicle_speed_mph)
                                    leading_vehicle_speed_mph = current_leading_vehicle_speed_mph;
                            }
                        }
                    }



                    // Handle lane changes
                    StateInput stateInput = StateInput(&vehicleTracker, lane, car_s, car_d, car_speed, LANE_WIDTH);
                    StateOutput stateOutput = stateMachine.runOnce(stateInput);
                    lane = stateOutput.nextLane;
                    extraLaneToMonitor = stateOutput.extraLaneToMonitor;


                    // Apply target velocity
                    if (leading_vehicle_detected) {
                        target_vel = leading_vehicle_speed_mph;
                    } else {
                        target_vel = TARGET_VELOCITY;
                    }

                    // Apply delta to reference velocity
                    if (inRange(target_vel - .5, target_vel + .5, ref_vel))
                        ref_vel = target_vel;
                    else {
                        if (ref_vel < target_vel)
                            ref_vel += MAX_DELTA_V;
                        else
                            ref_vel -= MAX_DELTA_V;
                    }

                    /*  PATH GENERATION */

                    // create a list of widley spaced (x,y) points, evenly spread at 30m
                    // later we will interpolate these waypoints with a spline and fit it in with more points that control speed
                    vector<double> ptsx;
                    vector<double> ptsy;

                    // reference x,y, ywa states
                    // either we will reference the starting point as where the car is at the previous paths end point
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);


                    // if previous size is almost empty, use the car as starting reference
                    if (prev_size < 2) {
                        // This gets called only when starts
                        // use 2 points that make the path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    } else {// use the previous path's end point as reference
                        // redefine reference state as previous path end point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // Use two points that make the path tangent to the previous path's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }


                    if (prev_size > 0)
                        car_s = end_path_s;

                    // In Frenet add evenly spaced points ahead of the 30m starting reference (works good in the long
                    // run, but it is a bit worse when starting in a curve, it will not estimate it as well
                    for (int i = 0; i < TRAJECTORY_POINT_COUNT; i++) {
                        vector<double> next_wp = getXY(car_s + 30 + i * TRAJECTORY_SPACING, 2 + LANE_WIDTH * lane,
                                                       map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        ptsx.push_back(next_wp[0]);
                        ptsy.push_back(next_wp[1]);
                    }


                    // shift back points to original reference system
                    for (int i = 0; i < ptsx.size(); i++) {
                        // shift the reference angle to 0 degrees (avoids problems with spline function becoming vertical)
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }


                    //create a spline
                    tk::spline s;

                    //set (x,y) points to the spline
                    s.set_points(ptsx, ptsy);

                    //Define the actual (x,y) we will use for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


                    //Start with all the previous path points from last time
                    int previous_path_size = min(prev_size, PATH_MAX_POINTS);
                    for (int i = 0; i < previous_path_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    //Calculate how to break up spline points so that we travel at our desired reference velocity
                    double target_x = 40;   //m
                    double target_y = s(target_x);
                    double target_dist = sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on = 0;

                    //Fill up the rest of our path planner after filling it with previous points
                    for (int i = 0; i <= PATH_MAX_POINTS - previous_path_size; i++) {
                        double N = (target_dist / (.02 * ref_vel / 2.23694));
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        //rotate back to normal after earlier rotation
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
