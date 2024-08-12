#include "single_agent_waypoint_following/PID_controller.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

PIDController::PIDController(rclcpp::Node* node, size_t max_cycles)
    : node_(node), tf_buffer_(), tf_listener_(tf_buffer_)
{
    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.description = "Constant speed for the TurtleBot3";

    node_->declare_parameter<double>("speed", DEFAULT_SPEED, speed_descriptor);
    this->set_parameters();

    linear_vel_ = {0, 0, 0};
    angular_vel_ = {0, 0, 0};

    goal_status_ = EMPTY;
    total_cycles_ = 0;
    max_cycles_ = max_cycles;

    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&PIDController::param_change_callback, this, std::placeholders::_1));
    
    // service for default speed setting
    this->speed_service_ = node_->create_service<speed_interface::srv::SetSpeed>("set_speed", std::bind(&PIDController::set_speed, this, _1));

    timer_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_ = node_->create_wall_timer(100ms, std::bind(&PIDController::control_cycle, this), timer_cb_group);
    this->timer_->cancel();

    // Control velocity publisher
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // this->vel_timer_ = node_->create_wall_timer(10ms, std::bind(&PIDController::send_velocity, node_));
    // this->pose_timer_ = node_->create_wall_timer(150ms, std::bind(&PIDController::publish_pose, node_));
}

PIDController::GoalStatus PIDController::get_goal_status()
{
    return goal_status_;
}

PIDController::GoalResult PIDController::get_goal_result()
{
    return goal_result_;
}

void PIDController::enable()
{
    // this->enabled_ = true;
    goal_status_ = ONGOING;
    this->timer_->reset();
}

void PIDController::reset()
{
    total_cycles_ = 0;
    goal_status_ = EMPTY;
}

double PIDController::get_performance()
{
    rclcpp::Rate rate(1s);

    geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr = nullptr;
    while ((odom2robot_ptr = get_position()) == nullptr)
        rate.sleep();
    auto odom2robot = *(odom2robot_ptr.get());

    double x = odom2robot.transform.translation.x;
    double y = odom2robot.transform.translation.y;

    double dist = std::sqrt(std::pow(goal_.x - x, 2) + std::pow(goal_.y - y, 2));
    
    RCLCPP_DEBUG(node_->get_logger(), "Performance: {%f}", total_cycles_ + dist);

    return total_cycles_ + dist;
}

void PIDController::stop_robot()
{
    linear_vel_.setX(0);
    linear_vel_.setY(0);
    angular_vel_.setZ(0);
    send_velocity();
}

void PIDController::set_parameters()
{
    this->speed_ = node_->get_parameter("speed").as_double();
}

void PIDController::set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request> request)
{
    if (request->speed >= MAX_SPEED)
    {
        RCLCPP_WARN(node_->get_logger(), "Setting maximum supported speed: %f. Consider switching to lower speed to avoid operating your robot at critical conditions.", MAX_SPEED);
        RCLCPP_WARN(node_->get_logger(), "Robot might behave in unstable manner. Consider increasing the tolerances if you want to run at maximum speed.");
        this->speed_ = MAX_SPEED;
    }
    else
        this->speed_ = request->speed;
    RCLCPP_INFO(node_->get_logger(), "Set speed to: {%f}", this->speed_);
}

// used for setpoints initialization
double PIDController::normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

void PIDController::set_goal(Setpoint setpoint)
{
    this->goal_ = setpoint;
}

geometry_msgs::msg::TransformStamped::SharedPtr PIDController::get_position()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(node_->get_logger(), "Odom to robot transform not found: %s", e.what());
        return nullptr;
    }
    return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(odom2robot));
}

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

void PIDController::control_cycle()
{
    total_cycles_++;
    if (total_cycles_ == max_cycles_)
    {
        stop_robot();
        goal_result_ = FAIL;
        goal_status_ = FINISHED;
        this->timer_->cancel();
        RCLCPP_DEBUG(node_->get_logger(), "Goal fail!");
        return;
    }

    static double old_vel = 0;
    auto odom2robot_ptr = get_position();
    if (odom2robot_ptr == nullptr)
        return;
    auto odom2robot = *(odom2robot_ptr.get());

    double x = odom2robot.transform.translation.x;
    double y = odom2robot.transform.translation.y;

    orientation_.setX(odom2robot.transform.rotation.x);
    orientation_.setY(odom2robot.transform.rotation.y);
    orientation_.setZ(odom2robot.transform.rotation.z);
    orientation_.setW(odom2robot.transform.rotation.w);
    
    double yaw;
    yaw = tf2::getYaw(orientation_);
    
    RCLCPP_DEBUG(node_->get_logger(), "x,y: {%f %f}", x, y);
    RCLCPP_DEBUG(node_->get_logger(), "Desired position: {%f %f}", goal_.x, goal_.y);

    double x_diff = goal_.x - x;
    double y_diff = goal_.y - y;
    double yaw_desired = atan2(y_diff, x_diff);
    double yaw_diff = angles::shortest_angular_distance(yaw, yaw_desired);

    double vel_x = get_linear_velocity(x_diff, y_diff);
    old_vel = vel_x;
    linear_vel_.setX(vel_x);
    if (vel_x == 0)
    {
        angular_vel_.setZ(0);
        // applying tiny back acceleration burst to weaken simulated robot's spontaneous drift after turning
        linear_vel_.setX(-1*old_vel*K_l);
        send_velocity();
        linear_vel_.setX(0);
        send_velocity();
        goal_result_ = SUCCESS;
        goal_status_ = FINISHED;
        timer_->cancel();
        RCLCPP_DEBUG(node_->get_logger(), "Goal success!");
    }
    else
    {
        double vel_theta = get_angular_velocity(yaw_diff);
        angular_vel_.setZ(vel_theta);
        send_velocity();
    }
}

void PIDController::send_velocity()
{
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear = tf2::toMsg(linear_vel_);
    cmd_vel.angular = tf2::toMsg(angular_vel_);
    this->vel_pub_->publish(cmd_vel);
}

// callback for setting default speed via a ROS parameter
rcl_interfaces::msg::SetParametersResult PIDController::param_change_callback(const std::vector<rclcpp::Parameter> &params)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &param : params) 
    {
        // checking if the changed parameter is of the right type
        if (param.get_name() == "speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
        {
            if (param.as_double() >= MAX_SPEED)
            {
                RCLCPP_WARN(node_->get_logger(), "Setting maximum supported speed: %f. Consider switching to lower speed to avoid operating your robot at critical conditions.", param.as_double());
                RCLCPP_WARN(node_->get_logger(), "Robot might behave in unstable manner. Consider increasing the tolerances if you want to run at maximum speed.");
                this->speed_ = MAX_SPEED;
            }
            else {
                this->speed_ = param.as_double();
                RCLCPP_INFO(node_->get_logger(), "Parameter 'speed' has changed. The new value is: %f", param.as_double());
            }
        } 
        else
        {
            result.successful = false;
            result.reason = "Unsupported parameter";
        }
    } 
    return result;
}