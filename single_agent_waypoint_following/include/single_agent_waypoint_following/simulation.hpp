#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "PID_controller.hpp"

class Simulation : public rclcpp::Node
{
    public:
        explicit Simulation(size_t, size_t, double, const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "NEAT_evolution");
        void simulation_run();
        double get_fitness();
        // bool is_running();

    private:
        PIDController::Setpoint generate_setpoint();

        // private data
        // bool running_;
        size_t runs_;
        double radius_;

        std::shared_ptr<PIDController> controller_;
        
        // coordinates transform
        // tf2::BufferCore tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        std::vector<double> points_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group;
};

#endif
