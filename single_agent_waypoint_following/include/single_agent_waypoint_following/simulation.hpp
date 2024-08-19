#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "agent_interface/srv/set_speed.hpp"
#include "agent_interface/srv/set_config.hpp"
#include "agent_interface/action/setpoint.hpp"
#include "agent_interface/msg/stats.hpp"

class Simulation : public rclcpp::Node
{
    // TMP
    const size_t P_ = 10;
    const double R = 3.0;
    const size_t S = 5; // simulation runs

    using Stats = agent_interface::msg::Stats;
    using Setpoint = agent_interface::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ClientGoalHandle<Setpoint>;

    public:
        explicit Simulation(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "NEAT_simulator");
        void simulation_run();
        double get_fitness();

    private:
        geometry_msgs::msg::Point generate_setpoint();
        // void goal_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr>);
        void goal_response_callback(std::shared_ptr<GoalHandleSetpoint> future);
        void feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback>);
        void result_callback(const GoalHandleSetpoint::WrappedResult&);
        double get_distance(geometry_msgs::msg::Point&, geometry_msgs::msg::Point&);
        geometry_msgs::msg::TransformStamped::SharedPtr get_position();

        // private data
        bool finished_;
        size_t runs_;
        double radius_;

        // std::shared_ptr<Controller> controller_;
        Stats current_evaluation_;
        
        // coordinates transform
        // tf2::BufferCore tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        std::vector<Stats> points_;

        rclcpp::Client<agent_interface::srv::SetSpeed>::SharedPtr speed_client_;
        rclcpp::Client<agent_interface::srv::SetConfig>::SharedPtr config_client_;
        rclcpp_action::Client<Setpoint>::SharedPtr action_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group;

        // coordinates transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};

#endif
