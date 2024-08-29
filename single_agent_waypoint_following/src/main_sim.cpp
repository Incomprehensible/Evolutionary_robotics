#include "single_agent_waypoint_following/simulation.hpp"

// using MyGenome = PGenome<double, 2>;
#define NUM_INPUTS 3
#define NUM_OUTPUTS 2
#define NUM_HIDDEN 2
#define NUM_IN_HIDDEN 10
// #define WEIGHTS (((NUM_INPUTS+1)*NUM_OUTPUTS))
#define WEIGHTS (((NUM_INPUTS+1)*NUM_IN_HIDDEN) + ((NUM_HIDDEN-1)*NUM_IN_HIDDEN*(NUM_IN_HIDDEN+1)) + (NUM_OUTPUTS*(NUM_IN_HIDDEN+1)))

using MyGenome = CNNGenome<double, WEIGHTS>;
RCLCPP_COMPONENTS_REGISTER_NODE(Simulation<MyGenome>)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto config = std::make_shared<agent_interface::srv::SetConfig::Request>();
    config->cnnconf.num_inputs = NUM_INPUTS;
    config->cnnconf.num_outputs = NUM_OUTPUTS;
    config->cnnconf.num_hidden = NUM_HIDDEN;
    config->cnnconf.neurons_per_hidden = NUM_IN_HIDDEN;
    config->cnnconf.weights = {};

    auto simulator = std::make_shared<Simulation<MyGenome>>(config);
    // rclcpp::Logger logger = simulator->get_logger();
    // logger.set_level(rclcpp::Logger::Level::Debug);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(simulator);
    executor.spin();

    // rclcpp::spin(simulator);

    rclcpp::shutdown();

    return 0;
}