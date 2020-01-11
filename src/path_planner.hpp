#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "helpers.h"

void straight_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_x,
    const double car_y,
    const double car_yaw);

void circle_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const vector<double> &previous_path_x,
    const vector<double> &previous_path_y,
    const double car_x,
    const double car_y,
    const double car_yaw);

void along_lane_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_s,
    const double car_d,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y);

void spline_along_lane_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_x,
    const double car_y,
    const double car_yaw,
    const double car_speed,
    const vector<double> &previous_path_x,
    const vector<double> &previous_path_y,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y);
#endif