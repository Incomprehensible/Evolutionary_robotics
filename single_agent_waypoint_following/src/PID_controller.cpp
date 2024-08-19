#include "single_agent_waypoint_following/PID_controller.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

// PIDController::PIDController(rclcpp::Node* node)
//     : Controller(node)
// PIDController::PIDController() : Controller()
PIDController::PIDController(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Controller(options, node_name)
{
    total_cycles_ = 0;
    // max_cycles_ = max_cycles;

    // timer_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // this->timer_ = node_->create_wall_timer(100ms, std::bind(&PIDController::control_cycle, this), timer_cb_group);
    // this->timer_->cancel();
}

// PIDController::GoalStatus PIDController::get_goal_status()
// {
//     return goal_status_;
// }

// PIDController::GoalResult PIDController::get_goal_result()
// {
//     return goal_result_;
// }

// void PIDController::enable()
// {
//     goal_status_ = ONGOING;
//     this->timer_->reset();
// }

// void PIDController::reset()
// {
//     total_cycles_ = 0;
//     goal_status_ = EMPTY;
// }

void PIDController::set_stats()
{
    // rclcpp::Rate rate(1s);

    // geometry_msgs::msg::Pose::SharedPtr pose_ptr = nullptr;
    // while ((pose_ptr = get_position()) == nullptr)
    //     rate.sleep();
    // auto odom2robot = *(odom2robot_ptr.get());

    // double x = odom2robot.transform.translation.x;
    // double y = odom2robot.transform.translation.y;
    assert(current_pose_ != nullptr);
    double x = current_pose_->position.x;
    double y = current_pose_->position.y;

    stats_.distance_to_goal = std::sqrt(std::pow(goal_.x - x, 2) + std::pow(goal_.y - y, 2));
    // stats_.distance_traveled = total_dist_;
    stats_.total_cycles = total_cycles_;
    stats_.speed = speed_;
    
    RCLCPP_DEBUG(node_->get_logger(), "Performance: {%ld, %f}", total_cycles_, stats_.distance_to_goal);
}


// used for setpoints initialization
double PIDController::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

// void PIDController::set_goal(Simulation::Setpoint setpoint)
// {
//     this->goal_ = setpoint;
// }

double PIDController::get_linear_velocity(double x, double y) {
    double vel_x = 0;
    double dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    // RCLCPP_DEBUG(node_->get_logger(), "Linear distance: %f", dist);

    if (abs(dist) > distanceTolerance) {
        vel_x = K_l * dist;
        if (abs(vel_x) > this->speed_)
            vel_x = (dist > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_x;
}

// heading angle control
double PIDController::get_angular_velocity(double theta) {
    double vel_theta = 0;

    // RCLCPP_DEBUG(node_->get_logger(), "Angular distance: %f", theta);

    if (abs(theta) > headingAngleTolerance) {
        vel_theta = K_ha * theta;
        if (abs(vel_theta) > this->speed_)
            vel_theta = (theta > 0)? this->speed_ : this->speed_*-1;
    }
    return vel_theta;
}

void PIDController::go_to_setpoint()
{
    // static double old_vel = 0.0;
    // in case we don't have a timer
    if (!enabled_)
        return;
    total_cycles_++;

    // auto pose_ptr = get_position();
    // static_assert(pose_ptr != nullptr);
    // if (pose_ptr == nullptr)
    //     return;
    // auto pose = *(pose_ptr.get());
    assert(current_pose_ != nullptr);
    auto pose = *(current_pose_.get());

    double x = pose.position.x;
    double y = pose.position.y;

    orientation_.setX(pose.orientation.x);
    orientation_.setY(pose.orientation.y);
    orientation_.setZ(pose.orientation.z);
    orientation_.setW(pose.orientation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);
    
    RCLCPP_DEBUG(node_->get_logger(), "x,y: {%f %f}", x, y);
    RCLCPP_DEBUG(node_->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

    double x_diff = goal_.x - x;
    double y_diff = goal_.y - y;
    double yaw_desired = atan2(y_diff, x_diff);
    double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

    double vel_x = get_linear_velocity(x_diff, y_diff);
    // old_vel = vel_x;
    linear_vel_.setX(vel_x);
    if (vel_x == 0)
    {
        // angular_vel_.setZ(0);
        // // applying tiny back acceleration burst to weaken simulated robot's spontaneous drift after turning
        // linear_vel_.setX(-1*old_vel*K_l);
        // send_velocity();
        // linear_vel_.setX(0);
        // send_velocity();
        stop_robot();
        stats_.reached = true;
        
        RCLCPP_DEBUG(node_->get_logger(), "Goal success!");
    }
    else
    {
        double vel_theta = get_angular_velocity(yaw_diff);
        angular_vel_.setZ(vel_theta);
        send_velocity();
    }
    set_stats();
}

RCLCPP_COMPONENTS_REGISTER_NODE(PIDController)