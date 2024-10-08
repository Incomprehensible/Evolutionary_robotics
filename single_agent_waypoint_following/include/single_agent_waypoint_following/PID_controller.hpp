#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "controller.hpp"

class PIDController: public Controller
{
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
        void build_phenotype() override;

        // private data
        double K_l_;
        double K_ha_;
        // size_t total_cycles_;
        geometry_msgs::msg::PoseStamped old_pose_;
        double old_yaw_;
        double old_vel_theta_;
        double old_alpha_theta_;
};

#endif
