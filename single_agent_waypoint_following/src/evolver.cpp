#include "single_agent_waypoint_following/evolver.hpp"
#include "single_agent_waypoint_following/controller.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>

// contain multiple genome candidates
// can produce new populations and create corresponding phenotypes for evaluation

Evolver::Evolver(const rclcpp::NodeOptions & options, const std::string & node_name)
    : Node(node_name, options), tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
{
    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.description = "Constant speed for the TurtleBot3";

    node_->declare_parameter<double>("speed", DEFAULT_SPEED, speed_descriptor);
    this->set_parameters();
}

void Evolver::set_speed(const std::shared_ptr<speed_interface::srv::SetSpeed::Request> request)
{
    this->speed_ = request->speed;
}