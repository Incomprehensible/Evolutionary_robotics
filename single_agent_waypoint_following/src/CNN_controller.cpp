#include "single_agent_waypoint_following/CNN_controller.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>
#include <float.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

CNNController::CNNController(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Controller(options, node_name), cnn_(nullptr)
{
    reset_bookkeeping();
}

void CNNController::reset_bookkeeping()
{
    old_pose_ = geometry_msgs::msg::PoseStamped();
    old_yaw_ = 0.0;
    old_vel_x_ = 0.0;
    old_vel_theta_ = 0.0;
    old_alpha_theta_ = 0.0;
}

void CNNController::build_phenotype()
{
    if (config_ != nullptr)
    {
        cnn_ = std::make_shared<CNN>(config_->cnnconf.num_inputs, config_->cnnconf.num_outputs, 
            config_->cnnconf.num_hidden, config_->cnnconf.neurons_per_hidden, config_->cnnconf.weights);
    }
}

// move to Controller?
void CNNController::set_stats()
{
    assert(current_pose_ != nullptr);
    double x = current_pose_->pose.position.x;
    double y = current_pose_->pose.position.y;

    stats_.distance_to_goal = std::sqrt(std::pow(goal_.x - x, 2) + std::pow(goal_.y - y, 2));
    stats_.distance_traveled = total_dist_;
    stats_.total_cycles = total_cycles_;
    stats_.total_rotations = total_rot_;
    stats_.total_jerk = total_jerk_;
    stats_.total_lin_speed += std::abs(old_vel_x_);
    stats_.lin_speed = old_vel_x_;
    stats_.ang_speed = old_vel_theta_;
    
    RCLCPP_DEBUG(node_->get_logger(), "Performance: {%ld, %f}", total_cycles_, stats_.distance_to_goal);
}


// used for setpoints initialization
double CNNController::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

inline bool is_zero(double x)
{
    return std::fabs(x) <= DBL_EPSILON;
}

void CNNController::go_to_setpoint()
{
    // in case we don't have a timer
    assert(enabled_ == true);

    total_cycles_++;

    assert(current_pose_ != nullptr);
    auto pose_stamped = *(current_pose_.get());
    auto pose = pose_stamped.pose;

    double x = pose.position.x;
    double y = pose.position.y;

    if (!is_zero(old_pose_.pose.position.x) && !is_zero(old_pose_.pose.position.y))
        total_dist_ += std::sqrt(std::pow(x - old_pose_.pose.position.x, 2) + std::pow(y - old_pose_.pose.position.y, 2));

    orientation_.setX(pose.orientation.x);
    orientation_.setY(pose.orientation.y);
    orientation_.setZ(pose.orientation.z);
    orientation_.setW(pose.orientation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);

    if (!is_zero(old_yaw_))
        total_rot_ += std::abs(yaw-old_yaw_);
    
    RCLCPP_DEBUG(node_->get_logger(), "x,y: {%f %f}", x, y);
    RCLCPP_DEBUG(node_->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

    double x_diff = goal_.x - x;
    double y_diff = goal_.y - y;
    double yaw_desired = atan2(y_diff, x_diff);
    double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

    double vel_x;
    double vel_theta;
    // tmp
    double dist = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
    if (abs(dist) <= distanceTolerance) {
        // stop_robot();
        stats_.reached = true;
        
        RCLCPP_DEBUG(node_->get_logger(), "Goal success!");
    }
    // else {
        std::vector<double> inputs = {x_diff, y_diff, yaw_diff};

        assert(cnn_ != nullptr);
        auto outputs = cnn_->update(inputs);

        vel_x = outputs[0] * MAX_SPEED;
        vel_theta = outputs[1] * MAX_SPEED;

        if (abs(vel_x) > this->speed_)
            vel_x = (vel_x > 0)? this->speed_ : this->speed_*-1;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (vel_theta > 0)? this->speed_ : this->speed_*-1;

        RCLCPP_DEBUG(node_->get_logger(), "vel_x:{%f}, vel_theta:{%f}", vel_x, vel_theta);

        linear_vel_.setX(vel_x);
        angular_vel_.setZ(vel_theta);
        send_velocity();

        // doesn't work - we dont have goal success
        // if (is_zero(vel_x) == 0 && is_zero(vel_theta))
        // {
        //     stop_robot();
        //     stats_.reached = true;

        //     RCLCPP_DEBUG(node_->get_logger(), "Goal success!");
        // }
    // }

    double dt = (pose_stamped.header.stamp.sec-old_pose_.header.stamp.sec) + (pose_stamped.header.stamp.nanosec-old_pose_.header.stamp.nanosec)/1e9;
    double alpha_theta = (angular_vel_.getZ() - old_vel_theta_) / dt;
        // total_jerk_ += std::abs((angular_vel_.getZ()-old_vel_theta_)/std::pow(dt, 2));
    total_jerk_ += std::abs((alpha_theta - old_alpha_theta_) / dt);
    RCLCPP_DEBUG(node_->get_logger(), "total_jerk: {%f}", total_jerk_);
    RCLCPP_DEBUG(node_->get_logger(), "dt: {%f}", dt);
    
    // update bookkeeping vars
    old_vel_x_ = vel_x;
    old_vel_theta_ = vel_theta;

    set_stats();
    old_yaw_ = yaw;
    old_pose_ = pose_stamped;
    old_alpha_theta_ = alpha_theta;
}
