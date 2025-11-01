#pragma once

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>
#include <tuple>
#include <limits>
#include <algorithm>

class DWAPlannerCustom : public rclcpp::Node {
 public:
  DWAPlannerCustom() : Node("dwa_planner_custom") {
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DWAPlannerCustom::odomCallback, this, std::placeholders::_1));
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DWAPlannerCustom::scanCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Custom DWA Planner Node started!");
  }

  void setGoal(double gx, double gy) {
    goal_x_ = gx;
    goal_y_ = gy;
  }

  // Returns: (best_v, best_w, all_trajs)
  std::tuple<double, double, std::vector<std::vector<geometry_msgs::msg::Point>>> computeBestTrajectory()
  {
    double best_cost = std::numeric_limits<double>::max();
    double best_v = 0.0, best_w = 0.0;
    int best_idx = -1;

    std::vector<std::vector<geometry_msgs::msg::Point>> all_trajs;

    // Dynamic window limits based on current velocity and acceleration constraints
    double min_v = std::max(v_min_, curr_vx_ - acc_v * del_T);
    double max_v = std::min(v_max_, curr_vx_ + acc_v * del_T);
    double min_w = std::max(w_min_, curr_w_ - acc_w * del_T);
    double max_w = std::min(w_max_, curr_w_ + acc_w * del_T);

    for (double v = min_v; v <= max_v; v += v_res_) {           // Linear velocity samples with v_res
      for (double w = min_w; w <= max_w; w += w_res_) {         // Angular Velcoity samples  

        std::vector<geometry_msgs::msg::Point> traj;
        double sim_x = current_x;
        double sim_y = current_y;
        double sim_yaw = current_yaw;

        for (double t = 0; t < del_T; t += dt_) {
          sim_x += v * std::cos(sim_yaw) * dt_;
          sim_y += v * std::sin(sim_yaw) * dt_;
          sim_yaw += w * dt_;

          geometry_msgs::msg::Point p;
          p.x = sim_x;
          p.y = sim_y;
          p.z = 0.0;
          traj.push_back(p);
        }

      // ---- COST FUNCTIONS ----
        // 1. Heading cost (distance to goal at the end of trajectory)
        double dx = goal_x_ - traj.back().x;
        double dy = goal_y_ - traj.back().y;
        double heading_cost = std::hypot(dx, dy);

        // 2. Obstacle cost (minimum distance to obstacles along the trajectory)
        double obstacle_cost = 0.0;
        for (const auto &ob : obstacles_) {           // obstacles_ map
          for (const auto &p : traj) {                // trajectory rollouts
            double d = std::hypot(p.x - ob[0], p.y - ob[1]);
            if (d < inflation_rad) {
              obstacle_cost += (inflation_rad - d) * 10.0;
            }
          }
        }
        // 3. Velocity cost (higher forward velocities)
        double velocity_cost = (v_max_ - v);

        double total_cost = ALPHA * heading_cost + BETA * obstacle_cost + GAMMA * velocity_cost;

        all_trajs.push_back(traj);

        if (total_cost < best_cost) {
          best_cost = total_cost;
          best_v = v;
          best_w = w;
          best_idx = all_trajs.size() - 1;
        }
      }
    }

    // Move best trajectory to front
    if (best_idx > 0 && best_idx < (int)all_trajs.size()) {
      std::swap(all_trajs[0], all_trajs[best_idx]);
    }

    return {best_v, best_w, all_trajs};
  }

 private:
  // DWA parameters
  double v_min_ = 0.0;
  double v_max_ = 0.5;
  double w_min_ = -0.42;
  double w_max_ = 0.42;
  double acc_v = 0.2;
  double acc_w = 0.8;
  double v_res_ = 0.08;     // Linear velocity resolution(step size) 
  double w_res_ = 0.2;      // Angular velocity resolution(step size)
  double inflation_rad= 0.25;
  // Cost weights of DWA (usually ALPHA+BETA+GAMMA= close to 3)
  double ALPHA = 1.4;   // heading weight
  double BETA = 1.4;  // obstacle weight
  double GAMMA = 0.2;  // velocity weight

  double del_T = 2.0;   // Trajectory rollout
  double dt_ = 0.05;    // sampling frequency

  // States
  double current_x = 0.0;
  double current_y = 0.0;
  double current_yaw = 0.0;
  double curr_vx_ = 0.0;
  double curr_w_ = 0.0;
  double goal_x_=0.0;
  double goal_y_=0.0;

  std::vector<std::array<double, 2>> obstacles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    curr_vx_ = msg->twist.twist.linear.x;
    curr_w_ = msg->twist.twist.angular.z;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {   // obstacles_ map using LiDAR Point cloud
    std::vector<std::pair<double, double>> pts;
    double angle = msg->angle_min;

    for (const auto& r : msg->ranges)
    {
      if (r > msg->range_min && r < msg->range_max)
      {
        if (std::isnan(r) || std::isinf(r)) {
          angle += msg->angle_increment;
          continue;
        }
        double gx = current_x + r * std::cos(angle + current_yaw);   // robot frame to gloabal frame
        double gy = current_y + r * std::sin(angle + current_yaw);
        pts.emplace_back(gx, gy);
      }
      angle += msg->angle_increment;
    }
    obstacles_.clear();
    obstacles_.reserve(pts.size());
    for (const auto &pt : pts)
      obstacles_.push_back({pt.first, pt.second});
  }
};
