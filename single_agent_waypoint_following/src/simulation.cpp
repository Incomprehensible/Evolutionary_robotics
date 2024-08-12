#include <random>
#include "single_agent_waypoint_following/simulation.hpp"

using namespace std::chrono_literals;

Simulation::Simulation(size_t T, size_t S, double r, const rclcpp::NodeOptions & options, const std::string & node_name)
: Node(node_name, options)
{
    this->controller_ = std::make_shared<PIDController>(this, T);
    this->runs_ = S;
    this->radius_ = r;
    points_.resize(S);
    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_ = this->create_wall_timer(150ms, std::bind(&Simulation::simulation_run, this), timer_cb_group);
}

double Simulation::get_fitness()
{
    double f = 0;
    for (const auto& p : points_) {
        f += p;
    }
    f /= points_.size();
    return 1 / f;
}

void Simulation::simulation_run()
{
    rclcpp::Rate rate(1s);

    for (size_t i=0; i<runs_; ++i) {
        PIDController::Setpoint p = generate_setpoint();
        controller_->set_goal(p);
        controller_->enable();

        while ((controller_->get_goal_status()) != PIDController::FINISHED) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting...:");
            rate.sleep();
        }

        // define and use penalty after checking GoalResult
        points_.push_back(controller_->get_performance());
        controller_->reset();
    }
    RCLCPP_DEBUG(this->get_logger(), "Fitness: {%f}", get_fitness());
}

double generateEdgeBiasedRandom(double radius) {
    // Generate a uniform random number between 0 and 1
    std::random_device seed; // obtain a random number from hardware

    // distribution over closed interval
    std::uniform_real_distribution d(0.0, 1.0);
    auto gen = std::bind(d, std::mt19937(seed()));
    double u = gen();
    
    // Map the uniform random number to the U-shaped distribution
    // This transformation squares the random variable and adjusts its sign.
    double biased_value = std::copysign(std::sqrt(2 * radius * std::abs(u - 0.5)), u - 0.5);
    
    // Scale the biased value to fit the radius [-radius, radius]
    double random_number = biased_value * radius;

    return random_number;
}

PIDController::Setpoint Simulation::generate_setpoint()
{
    rclcpp::Rate rate(1s);

    geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
    // simulated robot is not spawned yet - wait
    while ((odom2robot_ptr = controller_->get_position()) == nullptr) {
        rate.sleep();
    }

    auto odom2robot = *(odom2robot_ptr.get());

    double x_origin = odom2robot.transform.translation.x;
    double y_origin = odom2robot.transform.translation.y;

    tf2::Quaternion orientation;

    orientation.setX(odom2robot.transform.rotation.x);
    orientation.setY(odom2robot.transform.rotation.y);
    orientation.setZ(odom2robot.transform.rotation.z);
    orientation.setW(odom2robot.transform.rotation.w);
    
    double yaw_origin = tf2::getYaw(orientation);

    // std::random_device seed; // obtain a random number from hardware
    // distribution over closed interval
    // std::uniform_real_distribution d(-radius_, radius_);
    // auto gen_row = std::bind(d_row, std::mt19937(seed));
    // auto gen = std::bind(d, std::mt19937(seed()));
    // randomly generate dx, dy, dyaw
    double dx = generateEdgeBiasedRandom(radius_);
    double dy = generateEdgeBiasedRandom(radius_);

    PIDController::Setpoint goal = {x_origin + dx, y_origin + dy, yaw_origin};
    return goal;
}
