#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#define DEBUG (1)

#if DEBUG
#include <iostream>
using std::cout;
using std::endl;
#endif

#include "helpers.h"
#include "spline.h"

enum State {
  INIT,
  ACC,
  SLOWDOWN
};
void straight_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_x,
    const double car_y,
    const double car_yaw)
{
  double dist_inc = 0.5;
  for (int i = 0; i < 50; ++i)
  {
    point_x.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    point_y.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
  }
}

void circle_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const vector<double> &previous_path_x,
    const vector<double> &previous_path_y,
    const double car_x,
    const double car_y,
    const double car_yaw)
{
  double pos_x, pos_y, angle;
  int path_size = previous_path_x.size();

  for (int i = 0; i < path_size; ++i)
  {
    point_x.push_back(previous_path_x[i]);
    point_y.push_back(previous_path_y[i]);
  }

  if (path_size == 0)
  {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
  }
  else
  {
    pos_x = previous_path_x[path_size - 1];
    pos_y = previous_path_y[path_size - 1];

    double pos_x2 = previous_path_x[path_size - 2];
    double pos_y2 = previous_path_y[path_size - 2];
    angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
  }

  double dist_inc = 0.5;
  for (int i = 0; i < 50 - path_size; ++i)
  {
    point_x.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100)));
    point_y.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100)));
    pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
    pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
  }
}

void along_lane_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_s,
    const double car_d,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y)
{
  double n_inc = 0.5;
  for(int i=0; i<50; ++i){
    double next_s = car_s + n_inc * i;
    double next_d = 6;
    vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    point_x.push_back(next_xy[0]);
    point_y.push_back(next_xy[1]);
  }
}

void spline_along_lane_planner(
    vector<double> &point_x,
    vector<double> &point_y,
    const double car_s,
    const double car_d,
    const double car_x,
    const double car_y,
    const double car_yaw,
    const double car_speed,
    const double end_path_s,
    const double end_path_d,
    const vector<double> &previous_path_x,
    const vector<double> &previous_path_y,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const vector<vector<double>> &sensor_fusion)
{
  int lane_index = 1;
  vector<double> pts_x, pts_y;
  double ref_s = car_s;
  double ref_d = car_d;
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  double ref_vel = car_speed;
  int prev_size = previous_path_x.size();
  
  if(prev_size > 0){
    ref_s = end_path_s;
    ref_d = end_path_d;
  }
  
  // check other cars using sensor fusion
  int state = ACC;
  for(int i=0; i<sensor_fusion.size(); ++i){
    double other_d = sensor_fusion[i][6];
    if(4 * lane_index < other_d && other_d < 4 * lane_index + 4){
      double other_vx = sensor_fusion[i][3];
      double other_vy = sensor_fusion[i][4];
      double other_s = sensor_fusion[i][5];
      double vel = sqrt(other_vx * other_vx + other_vy * other_vy);
      double other_s_future = other_s + vel * 0.02 * prev_size;
      if(ref_s < other_s_future && other_s_future - ref_s < 30.){
        state = SLOWDOWN;
      }
    }
  }

  // increase speed
  switch(state){
    case ACC:
      ref_vel += 1.;
      if(ref_vel > 48.)
        ref_vel = 48.;
      break;
    case SLOWDOWN:
      ref_vel -= 1.;
      break;
    default:
      break;
  }

  // make the first two points for the spline
  if(prev_size < 2){
    // make the path tangent to the car by creating two points
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    pts_x.push_back(prev_car_x);
    pts_x.push_back(car_x);
    pts_y.push_back(prev_car_y);
    pts_y.push_back(car_y);
  }
  else{
    // make the path tangent to the car using the last two points of the previous path
    double car_x_p2 = previous_path_x[prev_size - 2];
    double car_y_p2 = previous_path_y[prev_size - 2];
    double car_x_p1 = previous_path_x[prev_size - 1];
    double car_y_p1 = previous_path_y[prev_size - 1];
    pts_x.push_back(car_x_p2);
    pts_x.push_back(car_x_p1);
    pts_y.push_back(car_y_p2);
    pts_y.push_back(car_y_p1);
    ref_x = car_x_p1;
    ref_y = car_y_p1;
    ref_yaw = atan2(car_y_p1 - car_y_p2, car_x_p1 - car_x_p2);
    ref_s = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y)[0];
  }

  //make three points
  for(int i=1; i<4; ++i){
    double next_i_s = ref_s + 30 * i;
    double next_i_d = 2 + 4 * lane_index;
    vector<double> next_i_waypoint = getXY(next_i_s, next_i_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    pts_x.push_back(next_i_waypoint[0]);
    pts_y.push_back(next_i_waypoint[1]);
  }

  //transform the global map coordinates to local car coordinates: shift and rotate
  for(int i=0; i<pts_x.size(); ++i){
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    double rotate_radius = 0. - ref_yaw;
    pts_x[i] = shift_x * cos(rotate_radius) - shift_y * sin(rotate_radius);
    pts_y[i] = shift_x * sin(rotate_radius) + shift_y * cos(rotate_radius);
  }

  //spline interpolations
  tk::spline s;
  s.set_points(pts_x, pts_y);

  //filling points using previous_path;
  for(int i=0; i<previous_path_x.size(); ++i){
    point_x.push_back(previous_path_x[i]);
    point_y.push_back(previous_path_y[i]);
  }

  //filling points using interpolated points
  double target_x = 30.;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  int n_point = target_dist / (0.02 * ref_vel * 0.44704); // 0.02 s per point, 50 miles per hour = 50 * 0.44704 meters per second
  double delta_x = target_x / n_point;
  for(int i=1; i<=50-previous_path_x.size(); ++i){
    double local_x = i * delta_x;
    double local_y = s(local_x);
    double global_x = local_x * cos(ref_yaw) - local_y * sin(ref_yaw) + ref_x;
    double global_y = local_x * sin(ref_yaw) + local_y * cos(ref_yaw) + ref_y;
    point_x.push_back(global_x);
    point_y.push_back(global_y);
  }
}

#endif
