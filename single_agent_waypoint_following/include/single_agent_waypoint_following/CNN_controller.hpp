#ifndef CNN_CONTROLLER_HPP
#define CNN_CONTROLLER_HPP

#include "controller.hpp"
#include "CNN.hpp"

class CNNController: public Controller
{
    // position tolerances
    const double distanceTolerance = 0.4;//0.1;
    const double headingAngleTolerance = 0.2; //0.05;
    
    public:
        CNNController(const rclcpp::NodeOptions& = rclcpp::NodeOptions(), const std::string& = "CNN_controller");
        CNNController(const double, const double);

    private:
        double normalize_angle(double);
        double get_linear_velocity(double, double);
        double get_angular_velocity(double);
        void go_to_setpoint() override;
        void set_stats() override;
        void reset_bookkeeping() override;
        void build_phenotype() override;

        // private data
        // size_t total_cycles_;
        std::shared_ptr<CNN> cnn_;
        geometry_msgs::msg::PoseStamped old_pose_;
        double old_yaw_;
        double old_vel_x_;
        double old_vel_theta_;
        double old_alpha_theta_;
};

#endif
