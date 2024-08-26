// #include <rclcpp/rclcpp.hpp>
// #include <memory>
#include "single_agent_waypoint_following/PID_controller.hpp"
#include "single_agent_waypoint_following/simulation.hpp"

using MyGenome = PGenome<double, 2>;
RCLCPP_COMPONENTS_REGISTER_NODE(Simulation<MyGenome>)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // const double radius = 3.0;
    // const size_t S = 10; // simulation runs

    auto simulator = std::make_shared<Simulation<MyGenome>>();
    rclcpp::Logger logger = simulator->get_logger();
    // logger.set_level(rclcpp::Logger::Level::Debug);

    auto controller = std::make_shared<PIDController>(1.0, 10.0);
    logger = controller->get_logger();
    // logger.set_level(rclcpp::Logger::Level::Debug);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(simulator);
    executor.add_node(controller);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}