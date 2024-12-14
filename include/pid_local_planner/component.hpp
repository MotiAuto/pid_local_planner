#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

#include "pid_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pid_local_planner
{
    class PIDLocalPlanner : public rclcpp::Node
    {
        public:
        explicit PIDLocalPlanner(const rclcpp::NodeOptions&option=rclcpp::NodeOptions());

        void target_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Time last_;
        PIDGain gain_;
        std::shared_ptr<PIDController> x_pid_;
        std::shared_ptr<PIDController> y_pid_;
        std::shared_ptr<PIDController> rotation_pid_;
        float output_max_, output_min_;

        geometry_msgs::msg::PoseStamped::SharedPtr current;
        geometry_msgs::msg::PoseStamped target;
        bool target_flag;
    };
}

#endif