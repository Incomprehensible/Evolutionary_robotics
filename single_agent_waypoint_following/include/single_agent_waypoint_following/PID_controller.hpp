#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "controller.hpp"

class PIDController: public Controller<agent_interface::srv::SetConfig>
{
    // P-controller gains
    // gain for the linear motion control
    // const double K_l = 1.0;
    // gain for the heading angle control
    // const double K_ha = 10.0;
    // gain for the turning angle control
    // const double K_ta = 0.9;

    // position tolerances
    const double distanceTolerance = 0.1;
    const double headingAngleTolerance = 0.05;
    // const double turnAngleTolerance = 0.01;
    
    public:
        PIDController(const rclcpp::NodeOptions& = rclcpp::NodeOptions(), const std::string& = "PID_controller_node");
        PIDController(const double, const double);

    private:
        double normalize_angle(double);
        double get_linear_velocity(double, double);
        double get_angular_velocity(double);
        void go_to_setpoint() override;
        void set_stats() override;
        void reset_bookkeeping() override;

        // private data
        // size_t total_cycles_;
        geometry_msgs::msg::PoseStamped old_pose_;
        double old_yaw_;
        double old_vel_theta_;
        double old_alpha_theta_;
};

#endif
