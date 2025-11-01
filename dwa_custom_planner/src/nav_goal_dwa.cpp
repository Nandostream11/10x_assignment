#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "dwa_custom_planner/srv/to_goal.hpp"
#include "dwa_custom_planner/dwa_planner_custom.hpp"

using namespace std::placeholders;

class NAVGOALDWA : public rclcpp::Node
{
 public:
    NAVGOALDWA() : Node("nav_goal_dwa")
    {   // Goal service
        service_ = this->create_service<dwa_custom_planner::srv::ToGoal>("togoal", std::bind(&NAVGOALDWA::goal_callback, this, _1, _2));
        // marker array for visualization of trajectory rollouts and best trajectory selected
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dwa_trajectories", 10);
        // to publish best v, w commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "DWA Service Node started. Waiting for goal...");
    }
 private:
    rclcpp::Service<dwa_custom_planner::srv::ToGoal>::SharedPtr service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    void goal_callback(const std::shared_ptr<dwa_custom_planner::srv::ToGoal::Request> request, 
        std::shared_ptr<dwa_custom_planner::srv::ToGoal::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", request->goal_x, request->goal_y);
        auto dwa = std::make_shared<DWAPlannerCustom>();
        dwa->setGoal(request->goal_x, request->goal_y);
        rclcpp::Rate rate(20.0);       // 10 Hz

        double dist_to_goal = std::numeric_limits<double>::max();

        while (rclcpp::ok() && dist_to_goal > 0.15 )
        {
            // Compute best velocity command and trajectories
            auto [best_v, best_w, all_trajs] = dwa->computeBestTrajectory();

            // Extract best trajectory (first)
            std::vector<geometry_msgs::msg::Point> best_traj;
            if (!all_trajs.empty())
                best_traj = all_trajs.front();

            // Compute distance to goal from the last point
            if (!best_traj.empty()) {
                auto last = best_traj.back();
                dist_to_goal = std::hypot(request->goal_x - last.x, request->goal_y - last.y);
            }
            // Publish visualization and command
            publishTrajectories(all_trajs, best_traj);
            publishCmdVel(best_v, best_w);
            RCLCPP_INFO(this->get_logger(),"v=%.2f w=%.2f dist=%.2f", best_v, best_w, dist_to_goal);

            rclcpp::spin_some(dwa);  // let subscriptions update odom/scan in header
            rate.sleep();            
        }
        // Stop the robot
        publishCmdVel(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        // Goal response 
        response->linear_vel = 0.0;
        response->angular_vel = 0.0;
        response->traj_points.clear();
    }

    void publishTrajectories(const std::vector<std::vector<geometry_msgs::msg::Point>> &all_trajs, 
        const std::vector<geometry_msgs::msg::Point> &best_traj)
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        for (const auto &traj : all_trajs)
        {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "odom";
            m.header.stamp = this->now();
            m.ns = "dwa_trajs";
            m.id = id++;
            m.type = m.LINE_STRIP;
            m.action = m.ADD;
            m.scale.x = 0.02;
            m.color.r = m.color.g = m.color.b = 0.5;
            m.color.a = 0.4;
            m.points = traj;
            arr.markers.push_back(m);
        }
        if (!best_traj.empty())
        {
            auto m = visualization_msgs::msg::Marker();
            m.header.frame_id = "odom";
            m.header.stamp = this->now();
            m.ns = "dwa_trajs";
            m.id = id++;
            m.type = m.LINE_STRIP;
            m.action = m.ADD;
            m.scale.x = 0.04;
            m.color.g = 1.0;
            m.color.a = 1.0;
            m.points = best_traj;
            arr.markers.push_back(m);
        }
        marker_pub_->publish(arr);
    }


    void publishCmdVel(double v, double w)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NAVGOALDWA>());
    rclcpp::shutdown();
    return 0;
}
