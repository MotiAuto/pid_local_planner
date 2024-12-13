#include "pid_local_planner/component.hpp"

namespace pid_local_planner
{
    PIDLocalPlanner::PIDLocalPlanner(const rclcpp::NodeOptions& option) : Node("PIDLocalPlanner", option)
    {
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose",
            0,
            std::bind(&PIDLocalPlanner::target_callback, this, _1)
        );

        current_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose",
            0,
            std::bind(&PIDLocalPlanner::current_callback, this, _1)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 0);

        this->declare_parameter("p_gain", 1.0);
        this->get_parameter("p_gain", gain_.p_gain);
        this->declare_parameter("i_gain", 0.1);
        this->get_parameter("i_gain", gain_.i_gain);
        this->declare_parameter("d_gain", 0.1);
        this->get_parameter("d_gain", gain_.d_gain);

        this->declare_parameter("max_limit", 1.0);
        this->get_parameter("max_limit", output_max_);
        this->declare_parameter("min_limit", -1.0);
        this->get_parameter("min_limit", output_min_);

        x_pid_ = std::make_shared<PIDController>();
        x_pid_->setConfig(gain_, output_max_, output_min_);
        y_pid_ = std::make_shared<PIDController>();
        y_pid_->setConfig(gain_, output_max_, output_min_);
        rotation_pid_ = std::make_shared<PIDController>();
        rotation_pid_->setConfig(gain_, output_max_, output_min_);

        target = nullptr;
        current = nullptr;

        RCLCPP_INFO(this->get_logger(), "Start PIDLocalPlanner!! p_gain:%.1lf, i_gain:%.1lf, d_gain:%.1lf, max_limit:%.1lf, min_limit:%.1lf",
            gain_.p_gain, gain_.i_gain, gain_.d_gain, output_max_, output_min_);
    }

    void PIDLocalPlanner::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target = msg;
    }

    void PIDLocalPlanner::current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current = msg;
        auto current_time = this->now();

        if(last_.nanoseconds() > 0)
        {
            rclcpp::Duration delta_time = current_time - last_;
            auto dt = delta_time.seconds();

            if(target != nullptr)
            {
                tf2::Quaternion t_q(target->pose.orientation.x, target->pose.orientation.y, target->pose.orientation.z, target->pose.orientation.w);
                tf2::Quaternion c_q(current->pose.orientation.x, current->pose.orientation.y, current->pose.orientation.z, current->pose.orientation.w);
                tf2::Matrix3x3 t_m(t_q);
                tf2::Matrix3x3 c_m(c_q);
                double target_yaw, current_yaw,roll, pitch;
                t_m.getRPY(roll, pitch, target_yaw);
                c_m.getRPY(roll, pitch, current_yaw);

                const auto target_x = target->pose.position.x;
                const auto target_y = target->pose.position.y;
                const auto current_x = current->pose.position.x;
                const auto current_y = current->pose.position.y;

                const auto x_vec = x_pid_->calc(target_x, current_x, dt);
                const auto y_vec = y_pid_->calc(target_y, current_y, dt);
                const auto rotation_vec = rotation_pid_->calc(target_yaw, current_yaw, dt);

                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = x_vec;
                cmd.linear.y = y_vec;
                cmd.angular.z = rotation_vec;

                publisher_->publish(cmd);
            }
        }

        last_ = current_time;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pid_local_planner::PIDLocalPlanner)