#include <random>
#include "single_agent_waypoint_following/simulation.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

Simulation::Simulation(const rclcpp::NodeOptions & options, const std::string & node_name)
: Node(node_name, options), tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
{
    // this->controller_ = std::make_shared<Controller>(this);
    // TMP
    this->runs_ = S;
    this->radius_ = R;

    this->finished_ = false;
    points_.resize(S);
    current_evaluation_ = Stats();

    this->client_ptr_ = rclcpp_action::create_client<Setpoint>(this, "go_to_setpoint");

    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_ = this->create_wall_timer(150ms, std::bind(&Simulation::simulation_run, this), timer_cb_group);
}

double Simulation::get_fitness()
{
    double f = 0;
    double fail_penalty = 10.0; // TEMPORARY

    for (const auto& stat : points_) {
        f += stat.distance_to_goal;
        f += stat.total_cycles;
        f += stat.reached? 0 : fail_penalty;
    }
    f /= points_.size();
    return 1 / f;
}

geometry_msgs::msg::TransformStamped::SharedPtr Simulation::get_position()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "Odom to robot transform not found: %s", e.what());
        return nullptr;
    }
    return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(odom2robot));
}

double Simulation::get_distance(geometry_msgs::msg::Point& p1, geometry_msgs::msg::Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// currently evaluates just one instance of Controller population
void Simulation::simulation_run()
{
    this->timer_->cancel();
    rclcpp::Rate rate(1s);
    double speed = 1.0; // TEMPORARY
    double allowance_time = 3.0; // TEMPORARY

    for (size_t i=0; i<runs_; ++i) {
        finished_ = false;

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Setpoint::Goal();
        goal_msg.setpoint = generate_setpoint();

        geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
        while ((odom2robot_ptr = get_position()) == nullptr)
            rate.sleep();
        auto odom2robot = *(odom2robot_ptr.get());
        geometry_msgs::msg::Point pose;
        pose.x = odom2robot.transform.translation.x;
        pose.y = odom2robot.transform.translation.y;
        pose.z = 0;
    
        double dist = get_distance(goal_msg.setpoint, pose);

        double timeout = dist / speed;
        auto timeout_duration = std::chrono::duration<double>(timeout + allowance_time);

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<Setpoint>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&Simulation::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Simulation::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Simulation::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // Wait for the goal to be accepted or rejected
        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
        //     continue;
        // }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
            continue;
        }

        auto start_time = std::chrono::steady_clock::now();

        while (!finished_) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time >= timeout_duration)
            {
                RCLCPP_DEBUG(this->get_logger(), "Goal timeout! Canceling goal...");
                auto cancel_future = this->client_ptr_->async_cancel_goal(goal_handle);
                // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
                // {
                //     RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
                //     rclcpp::shutdown();
                // }
                auto cancel_response = cancel_future.get();
                if (cancel_response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
                    RCLCPP_INFO(this->get_logger(), "Goal successfully canceled");
                else
                    RCLCPP_WARN(this->get_logger(), "Goal failed to cancel");

                break;
            }
            RCLCPP_DEBUG(this->get_logger(), "Waiting...:");
            rate.sleep();
        }
        while (!finished_) rate.sleep();

        if (!current_evaluation_.reached) {
            // define penalty for goal failure
            RCLCPP_DEBUG(this->get_logger(), "Goal fail!");
        }

        points_.push_back(current_evaluation_);
    }
    RCLCPP_DEBUG(this->get_logger(), "Fitness: {%f}", get_fitness());
}

void Simulation::goal_response_callback(std::shared_ptr<GoalHandleSetpoint> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void Simulation::feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback)
{ 
    RCLCPP_INFO(this->get_logger(), "reached: {x:%f, y:%f}", feedback->current_pose.position.x, feedback->current_pose.position.y);
}

void Simulation::result_callback(const GoalHandleSetpoint::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

    current_evaluation_ = result.result->evaluation;
    RCLCPP_INFO(this->get_logger(), "Final pose reached: {x:%f, y:%f}", result.result->pose.position.x, result.result->pose.position.y);
    finished_ = true;
}

// void Simulation::simulation_run()
// {
//     rclcpp::Rate rate(1s);

//     for (size_t i=0; i<runs_; ++i) {
//         Setpoint p = generate_setpoint();
//         controller_->set_goal(p);
//         controller_->enable();

//         while ((controller_->get_goal_status()) != PIDController::FINISHED) {
//             RCLCPP_DEBUG(this->get_logger(), "Waiting...:");
//             rate.sleep();
//         }

//         // define and use penalty after checking GoalResult
//         points_.push_back(controller_->get_performance());
//         controller_->reset();
//     }
//     RCLCPP_DEBUG(this->get_logger(), "Fitness: {%f}", get_fitness());
// }

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

geometry_msgs::msg::Point Simulation::generate_setpoint()
{
    rclcpp::Rate rate(1s);

    geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
    // geometry_msgs::msg::Pose::SharedPtr odom2robot_ptr;
    // simulated robot is not spawned yet - wait
    while ((odom2robot_ptr = get_position()) == nullptr) {
        rate.sleep();
    }

    auto odom2robot = *(odom2robot_ptr.get());

    double x_origin = odom2robot.transform.translation.x;
    double y_origin = odom2robot.transform.translation.y;
    // double x_origin = odom2robot.position.x;
    // double y_origin = odom2robot.position.y;

    tf2::Quaternion orientation;

    // for generating random orientation - not needed now
    // orientation.setX(odom2robot.orientation.x);
    // orientation.setY(odom2robot.orientation.y);
    // orientation.setZ(odom2robot.orientation.z);
    // orientation.setW(odom2robot.orientation.w);

    orientation.setX(odom2robot.transform.rotation.x);
    orientation.setY(odom2robot.transform.rotation.y);
    orientation.setZ(odom2robot.transform.rotation.z);
    orientation.setW(odom2robot.transform.rotation.w);
    
    double yaw_origin = tf2::getYaw(orientation);

    // randomly generate dx, dy, dyaw
    double dx = generateEdgeBiasedRandom(radius_);
    double dy = generateEdgeBiasedRandom(radius_);

    geometry_msgs::msg::Point goal;
    goal.x = x_origin + dx;
    goal.y = y_origin + dy;
    goal.z = 0;
    return goal;
}

RCLCPP_COMPONENTS_REGISTER_NODE(Simulation)