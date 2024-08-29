#include "single_agent_waypoint_following/PID_controller.hpp"
#include "single_agent_waypoint_following/CNN_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // auto controller = std::make_shared<PIDController>(1.0, 10.0);
    auto controller = std::make_shared<CNNController>();
    // rclcpp::Logger logger = controller->get_logger();
    // logger.set_level(rclcpp::Logger::Level::Debug);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller);
    executor.spin();

    // rclcpp::spin(controller);

    rclcpp::shutdown();

    return 0;
}