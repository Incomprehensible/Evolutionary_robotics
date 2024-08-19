#include "single_agent_waypoint_following/controller.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>
#include <angles/angles.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

template <typename T>
Controller<T>::Controller(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Node(node_name, options), tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
{
    // TMP
    node_ = this;

    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.description = "Constant speed for the TurtleBot3";

    node_->declare_parameter<double>("speed", DEFAULT_SPEED, speed_descriptor);
    this->set_parameters();

    enabled_ = false;

    current_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
    goal_ = geometry_msgs::msg::Point();
    goal_handle_ = nullptr;
    stats_ = {};

    linear_vel_ = {0, 0, 0};
    angular_vel_ = {0, 0, 0};

    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&Controller<T>::param_change_callback, this, std::placeholders::_1));

    // service for configuration
    this->config_service_ = node_->create_service<T>("set_config", std::bind(&Controller<T>::set_config, this, _1));
    
    // service for default speed setting
    this->speed_service_ = node_->create_service<agent_interface::srv::SetSpeed>("set_speed", std::bind(&Controller<T>::set_speed, this, _1));

    // timer_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // this->timer_ = node_->create_wall_timer(100ms, std::bind(&Controller::control_cycle, this), timer_cb_group);
    // this->timer_->cancel();

    // Odometry Subscriber
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Controller<T>::odom_callback, this, std::placeholders::_1));

    // Control velocity publisher
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->action_server_ = rclcpp_action::create_server<Setpoint>(
      node_,
      "go_to_setpoint",
      std::bind(&Controller<T>::handle_goal, this, _1, _2),
      std::bind(&Controller<T>::handle_cancel, this, _1),
      std::bind(&Controller<T>::handle_accepted, this, _1));
}

template <typename T>
void Controller<T>::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (/*!node_->get_clock()->ros_time_is_active() ||*/ !tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0))) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for valid simulation time and map frame");
        return;
    }
    // Transform current_pose to map frame
    try {
        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header = msg->header;
        odom_pose.pose = msg->pose.pose;
        geometry_msgs::msg::PoseStamped map_pose;
        tf_buffer_.transform(odom_pose, map_pose, "odom", tf2::durationFromSec(1.0));
        current_pose_ = std::make_shared<geometry_msgs::msg::Pose>(map_pose.pose);
        if (enabled_)
            control_cycle();
    } catch (const tf2::ExtrapolationException& ex) {
        RCLCPP_ERROR(node_->get_logger(), "Extrapolation error when transforming pose: %s", ex.what());
    }
}

template <typename T>
geometry_msgs::msg::Pose::SharedPtr Controller<T>::get_position()
{
    return current_pose_;
}

template <typename T>
void Controller<T>::stop_robot()
{
    linear_vel_.setX(0);
    linear_vel_.setY(0);
    angular_vel_.setZ(0);
    send_velocity();
}

template <typename T>
void Controller<T>::set_parameters()
{
    this->speed_ = node_->get_parameter("speed").as_double();
}

template <typename T>
void Controller<T>::set_speed(const std::shared_ptr<agent_interface::srv::SetSpeed::Request> request)
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

template <class T>
void Controller<T>::set_config(const std::shared_ptr<typename T::Request> request) {
    config_ = request->config;
}

template <typename T>
void Controller<T>::control_cycle()
{
    if (goal_handle_->is_canceling()) {
        stop_robot();
        RCLCPP_INFO(node_->get_logger(), "Goal canceled/failed");
        goto conclude_goal;
    }

    // does moving, stats set, reached check and set, stops robot
    go_to_setpoint();

    if (stats_.reached) {
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        goto conclude_goal;
    }
    return;

    conclude_goal:
        // if we put a timer instead, remove enabled_ and reset timer here 
        enabled_ = false;
        if (rclcpp::ok())
            send_result();
}

template <typename T>
rclcpp_action::GoalResponse Controller<T>::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Setpoint::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "Received goal request.");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename T>
rclcpp_action::CancelResponse Controller<T>::handle_cancel(const std::shared_ptr<GoalHandleSetpoint> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename T>
void Controller<T>::handle_accepted(const std::shared_ptr<GoalHandleSetpoint> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Controller::execute, this, _1), goal_handle}.detach();
}

template <typename T>
void Controller<T>::send_feedback()
{
    auto feedback = std::make_shared<Setpoint::Feedback>();
    feedback->current_pose = *(current_pose_.get());
    goal_handle_->publish_feedback(feedback);
    RCLCPP_INFO(node_->get_logger(), "Publish feedback");
}

// keep inner structure for stats bookkeeping
template <typename T>
void Controller<T>::send_result()
{
    auto result = std::make_shared<Setpoint::Result>();
    result->evaluation = stats_;
    result->pose = *(current_pose_.get());
    // auto & stats = result->evaluation;

    if (stats_.reached)
        goal_handle_->succeed(result);
    else
        goal_handle_->canceled(result);
}

template <typename T>
void Controller<T>::reset_goal()
{
    goal_ = geometry_msgs::msg::Point();
    goal_handle_.reset();
    stats_ = Stats();
}

template <typename T>
void Controller<T>::execute(const std::shared_ptr<GoalHandleSetpoint> goal_handle)
{
    reset_goal();
    RCLCPP_INFO(node_->get_logger(), "Executing goal");
    goal_handle_ = goal_handle;
    goal_ = goal_handle->get_goal()->setpoint;
    enabled_ = true;
    // if we put a timer instead, remove enabled_ and reset timer here 
    control_cycle();
  }

// REMOVED
// geometry_msgs::msg::TransformStamped::SharedPtr Controller::get_position()
// {
//     geometry_msgs::msg::TransformStamped odom2robot;

//     try {
//         odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
//     } catch (tf2::TransformException& e) {
//         RCLCPP_ERROR(node_->get_logger(), "Odom to robot transform not found: %s", e.what());
//         return nullptr;
//     }
//     return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(odom2robot));
// }

template <typename T>
void Controller<T>::send_velocity()
{
    geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
    cmd_vel.linear = tf2::toMsg(linear_vel_);
    cmd_vel.angular = tf2::toMsg(angular_vel_);
    this->vel_pub_->publish(cmd_vel);
}

// callback for setting default speed via a ROS parameter
template <typename T>
rcl_interfaces::msg::SetParametersResult Controller<T>::param_change_callback(const std::vector<rclcpp::Parameter> &params)
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
