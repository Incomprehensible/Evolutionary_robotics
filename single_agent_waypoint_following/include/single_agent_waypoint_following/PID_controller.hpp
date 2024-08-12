#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "speed_interface/srv/set_speed.hpp" 

// constants
#define SQUARE_POLYGON 5.0
#define DEFAULT_SPEED 1.0
#define MAX_SPEED 2.0

class PIDController
{
    // P-controller gains
    // gain for the linear motion control
    const double K_l = 1.0;
    // gain for the heading angle control
    const double K_ha = 10.0;
    // gain for the turning angle control
    const double K_ta = 0.9;

    // position tolerances
    const double distanceTolerance = 0.1;
    const double headingAngleTolerance = 0.05;
    const double turnAngleTolerance = 0.01;
    
    public:
        struct Setpoint {
            double x;
            double y;
            double yaw;
        };

        enum GoalResult {
            FAIL,
            SUCCESS
        };

        enum GoalStatus {
            EMPTY,
            ONGOING,
            FINISHED,
        };

        explicit PIDController(rclcpp::Node*, size_t);
        void set_parameters();
        void set_goal(Setpoint);
        GoalStatus get_goal_status();
        GoalResult get_goal_result();
        void reset();
        void enable();
        void disable();
        double get_performance();
        geometry_msgs::msg::TransformStamped::SharedPtr get_position();

    private:
        void set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request>);

        double normalize_angle(double);

        double get_linear_velocity(double, double);
        double get_angular_velocity(double);

        void control_cycle();
        void stop_robot();

        void send_velocity();
        rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter>&);

        // private data
        rclcpp::Node* node_;
        // bool enabled_;
        Setpoint goal_;
        GoalStatus goal_status_;
        GoalResult goal_result_;
        double speed_;
        size_t total_cycles_;
        size_t max_cycles_;
        tf2::Vector3 linear_vel_; 
        tf2::Vector3 angular_vel_;
        tf2::Quaternion orientation_;
        tf2::Vector3 position_;
        // timers
        rclcpp::TimerBase::SharedPtr timer_;
        // coordinates transform
        tf2::BufferCore tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        // publishers, subscribers, service
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        rclcpp::Service<speed_interface::srv::SetSpeed>::SharedPtr speed_service_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group;
};

#endif
