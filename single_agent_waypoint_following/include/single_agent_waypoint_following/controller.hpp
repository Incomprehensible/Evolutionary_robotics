#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "speed_interface/srv/set_speed.hpp" 
#include "goal_interface/action/setpoint.hpp"
#include "goal_interface/msg/stats.hpp"

#define MAX_SPEED 2.0
#define DEFAULT_SPEED 1.0

class Controller : public rclcpp::Node
{
    using Stats = goal_interface::msg::Stats;
    using Setpoint = goal_interface::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ServerGoalHandle<Setpoint>;

    public:
        explicit Controller(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "NEAT_controller");
        // explicit Controller(rclcpp::Node*);
        void set_parameters();
        geometry_msgs::msg::Pose::SharedPtr get_position();
        // void set_goal(Setpoint);
        // void set_goal(Simulation::Setpoint);
        // GoalStatus get_goal_status();
        // GoalResult get_goal_result();
        // virtual void reset() = 0;
        // virtual void enable() = 0;
        // virtual void disable() = 0;

    private:
        virtual void go_to_setpoint() = 0;
        virtual void set_stats() = 0;

        void reset_goal();
        void control_cycle();

        void set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request>);
        rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter>&);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr);

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Setpoint::Goal>);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetpoint>);
        void handle_accepted(const std::shared_ptr<GoalHandleSetpoint>);
        void execute(const std::shared_ptr<GoalHandleSetpoint>);
        void send_feedback();
        void send_result();

        // private data
        std::shared_ptr<GoalHandleSetpoint> goal_handle_;
        // publishers, subscribers, service
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp_action::Server<Setpoint>::SharedPtr action_server_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        rclcpp::Service<speed_interface::srv::SetSpeed>::SharedPtr speed_service_;
        // rclcpp::CallbackGroup::SharedPtr timer_cb_group;

    protected:
        void stop_robot();
        void send_velocity();

        // protected data
        rclcpp::Node* node_;
        bool enabled_; // in case we don't have a timer
        Stats stats_;
        // use <geometry_msgs::msg::Point>?
        // Simulation::Setpoint goal_;
        geometry_msgs::msg::Point goal_;
        geometry_msgs::msg::Pose::SharedPtr current_pose_;
        double speed_;
        tf2::Vector3 linear_vel_; 
        tf2::Vector3 angular_vel_;
        tf2::Quaternion orientation_;
        tf2::Vector3 position_;
        // coordinates transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};

#endif
