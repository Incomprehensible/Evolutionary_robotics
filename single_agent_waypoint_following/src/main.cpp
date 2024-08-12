#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "single_agent_waypoint_following/PID_controller.hpp"
#include "single_agent_waypoint_following/simulation.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    const size_t T = 50;
    const double radius = 3.0;
    const size_t S = 10; // simulation runs

    auto node = std::make_shared<Simulation>(T, S, radius);

    rclcpp::Logger logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Debug);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}