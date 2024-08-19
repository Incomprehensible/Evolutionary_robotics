#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "controller.hpp"

class PIDController: public Controller
{
    // P-controller gains
    // gain for the linear motion control
    const double K_l = 1.0;
    // gain for the heading angle control
    const double K_ha = 10.0;
    // gain for the turning angle control
    // const double K_ta = 0.9;

    // position tolerances
    const double distanceTolerance = 0.1;
    const double headingAngleTolerance = 0.05;
    // const double turnAngleTolerance = 0.01;
    
    public:

        // enum GoalResult {
        //     FAIL,
        //     SUCCESS
        // };

        // enum GoalStatus {
        //     EMPTY,
        //     ONGOING,
        //     FINISHED,
        // };

        // explicit PIDController(rclcpp::Node*);
        // explicit PIDController();
        PIDController(const rclcpp::NodeOptions& = rclcpp::NodeOptions(), const std::string& = "PID_controller_node");
        // GoalStatus get_goal_status();
        // GoalResult get_goal_result();
        // void reset();
        // void enable();
        // void disable();

    private:
        double normalize_angle(double);
        double get_linear_velocity(double, double);
        double get_angular_velocity(double);
        void go_to_setpoint() override;
        void set_stats() override;

        // private data
        // rclcpp::Node* node_;
        // bool enabled_;
        // Simulation::Setpoint goal_;
        // GoalStatus goal_status_;
        // GoalResult goal_result_;
        // double speed_;
        size_t total_cycles_;
        // size_t max_cycles_;
        // tf2::Vector3 linear_vel_; 
        // tf2::Vector3 angular_vel_;
        // tf2::Quaternion orientation_;
        // tf2::Vector3 position_;
        // timers
        // rclcpp::TimerBase::SharedPtr timer_;
        // coordinates transform
        // tf2::BufferCore tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        // publishers, subscribers, service
        // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        // rclcpp::Service<speed_interface::srv::SetSpeed>::SharedPtr speed_service_;
        // rclcpp::CallbackGroup::SharedPtr timer_cb_group;
};

#endif
