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

#include "evolver.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

template <class G>
class Simulation : public rclcpp::Node
{
    // TMP
    const size_t P_ = 10;
    const double R = 7.0; // 3.0;
    const size_t S = 8; // simulation runs
    const double F = 0.08;

    using Stats = agent_interface::msg::Stats;
    using Setpoint = agent_interface::action::Setpoint;
    using GoalHandleSetpoint = rclcpp_action::ClientGoalHandle<Setpoint>;

    public:
        explicit Simulation(const rclcpp::NodeOptions & = rclcpp::NodeOptions(), const std::string& = "NEAT_simulator");
        void simulation_run();
        double get_fitness();

    private:
        geometry_msgs::msg::Point generate_setpoint();
        std::vector<geometry_msgs::msg::Point> generate_setpoints();
        geometry_msgs::msg::Point generate_equidistant_setpoint();
        std::vector<geometry_msgs::msg::Point> generate_equidistant_setpoints();
        // void goal_response_callback(std::shared_future<GoalHandleSetpoint::SharedPtr>);
        void goal_response_callback(std::shared_ptr<GoalHandleSetpoint> future);
        void feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback>);
        void result_callback(const GoalHandleSetpoint::WrappedResult&);
        double get_distance(geometry_msgs::msg::Point&, geometry_msgs::msg::Point&);
        geometry_msgs::msg::TransformStamped::SharedPtr get_position();

        // private data
        Evolver<G> evolver_;
        bool finished_;

        // TMP
        size_t runs_;
        double radius_;
        double desired_fitness_;

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

template <class G>
Simulation<G>::Simulation(const rclcpp::NodeOptions & options, const std::string & node_name)
: Node(node_name, options), evolver_(P_, R*4, {1.0, 5.0}), tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_)
{
    // TMP
    this->runs_ = S;
    this->radius_ = R;
    this->desired_fitness_ = F;

    this->finished_ = false;
    points_.resize(S);
    current_evaluation_ = Stats();

    evolver_.init_population();

    this->action_client_ = rclcpp_action::create_client<Setpoint>(this, "go_to_setpoint");
    this->speed_client_ = this->create_client<agent_interface::srv::SetSpeed>("set_speed");
    this->config_client_ = this->create_client<agent_interface::srv::SetConfig>("set_config");

    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_ = this->create_wall_timer(150ms, std::bind(&Simulation<G>::simulation_run, this), timer_cb_group);
}

template <class G>
double Simulation<G>::get_fitness()
{
    double f = 0;
    double fail_penalty = 10.0; // TMP
    double time_penalty = 0.15; // TMP
    double dist_penalty = 10.0; // TMP
    double jerk_penalty = 1.5;
    // double travel_penalty = 10.0;

    for (const auto& stat : points_) {
        f += dist_penalty * stat.distance_to_goal;
        f += time_penalty * (stat.total_cycles / 10.0);
        f += 100.0 * stat.total_rotations / 2*M_PI;
        f += 10.0 * stat.distance_traveled / radius_; // for non-equidistant setpoints need to normalize!
        double jerk = stat.total_jerk / 10.0;
        f += (jerk > 3.0)? jerk * jerk_penalty : jerk;
        f += stat.reached? 0 : fail_penalty;
        RCLCPP_INFO(this->get_logger(), "dist_to_goal:{%f}, total_cycles:{%d}, distance_traveled:{%f}\ntotal_rotations:{%f}, total_jerk:{%f}, reached:{%d}", stat.distance_to_goal, stat.total_cycles, stat.distance_traveled, stat.total_rotations, stat.total_jerk, stat.reached);
    }
    f /= points_.size();
    return 1 / f;
}

template <class G>
std::vector<geometry_msgs::msg::Point> Simulation<G>::generate_setpoints()
{
    std::vector<geometry_msgs::msg::Point> p;
    p.resize(runs_);

    for (size_t i=0; i<runs_; ++i) {
        p[i] = utils::generate_setpoint(radius_);
    }

    return p;
}

template <class G>
std::vector<geometry_msgs::msg::Point> Simulation<G>::generate_equidistant_setpoints()
{
    std::vector<geometry_msgs::msg::Point> p;
    p.resize(runs_);

    for (size_t i=0; i<runs_; ++i) {
        p[i] = utils::generate_equidistant_setpoint(radius_);
    }

    return p;
}

template <class G>
geometry_msgs::msg::TransformStamped::SharedPtr Simulation<G>::get_position()
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

template <class G>
double Simulation<G>::get_distance(geometry_msgs::msg::Point& p1, geometry_msgs::msg::Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

template <class G>
void Simulation<G>::simulation_run()
{
    this->timer_->cancel();
    rclcpp::Rate rate(1s);
    double speed = 3.0; // TMP
    double allowance_time = 5.0; // TMP
    
    std::vector<G>& population = evolver_.get_population();
    // std::vector<Genome<double, 2>>& population = evolver_.get_population();
    assert(population.size() == P_);

    auto setpoints = generate_equidistant_setpoints();

    auto speed_req = std::make_shared<agent_interface::srv::SetSpeed::Request>();
    speed_req->speed = speed;
    while (!speed_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the speed service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Speed service not available, waiting again...");
    }
    speed_client_->async_send_request(speed_req);

    auto request = std::make_shared<agent_interface::srv::SetConfig::Request>();

    for (size_t n=0; n<P_; ++n) {
        G& p = population[n];
        // Genome<double, 2>& p = population[n];
    
        request->k_l = p.genes_[0].raw_value;
        request->k_ha = p.genes_[1].raw_value;
        while (!config_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the configuration service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Configuration service not available, waiting again...");
        }
        config_client_->async_send_request(request);

        for (size_t i=0; i<runs_; ++i) {
            finished_ = false;

            if (!this->action_client_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
            while ((odom2robot_ptr = get_position()) == nullptr)
                rate.sleep();
            auto odom2robot = *(odom2robot_ptr.get());
            geometry_msgs::msg::Point pose;
            pose.x = odom2robot.transform.translation.x;
            pose.y = odom2robot.transform.translation.y;
            pose.z = 0;
    
            auto goal_msg = Setpoint::Goal();
            goal_msg.setpoint.x = setpoints[i].x + pose.x;
            goal_msg.setpoint.y = setpoints[i].y + pose.y;
            goal_msg.setpoint.z = setpoints[i].z + pose.z;
            // goal_msg.setpoint = generate_setpoint();

            double dist = get_distance(goal_msg.setpoint, pose);

            double timeout = dist / speed;
            auto timeout_duration = std::chrono::duration<double>(timeout + allowance_time);

            RCLCPP_DEBUG(this->get_logger(), "Sending goal...");

            auto send_goal_options = rclcpp_action::Client<Setpoint>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&Simulation<G>::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&Simulation<G>::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&Simulation<G>::result_callback, this, _1);
            auto goal_handle_future = this->action_client_->async_send_goal(goal_msg, send_goal_options);

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
                    auto cancel_future = this->action_client_->async_cancel_goal(goal_handle);
                    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
                    // {
                    //     RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
                    //     rclcpp::shutdown();
                    // }
                    auto cancel_response = cancel_future.get();
                    if (cancel_response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
                        RCLCPP_DEBUG(this->get_logger(), "Goal successfully canceled");
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

            points_[i] = current_evaluation_;
            // points_.push_back(current_evaluation_);
        }
        double fitness = get_fitness();
        p.fitness = fitness;
        RCLCPP_INFO(this->get_logger(), "Candidate: K_l: {%f}, K_ha: {%f}, Fitness: {%f}", request->k_l, request->k_ha, fitness);
        // points_.clear(); // tmp
    }
    
    double best_fitness = evolver_.rank_candidates();
    RCLCPP_INFO(this->get_logger(), "Best Fitness: {%f}", best_fitness);

    if (best_fitness < desired_fitness_)
    {
        RCLCPP_INFO(this->get_logger(), "Generating new population...");
        evolver_.generate_new_population();
        return this->timer_->reset();
    }
}

template <class G>
void Simulation<G>::goal_response_callback(std::shared_ptr<GoalHandleSetpoint> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

template <class G>
void Simulation<G>::feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback)
{ 
    RCLCPP_DEBUG(this->get_logger(), "reached: {x:%f, y:%f}", feedback->current_pose.position.x, feedback->current_pose.position.y);
}

template <class G>
void Simulation<G>::result_callback(const GoalHandleSetpoint::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

    current_evaluation_ = result.result->evaluation;
    RCLCPP_DEBUG(this->get_logger(), "Final pose reached: {x:%f, y:%f}", result.result->pose.position.x, result.result->pose.position.y);
    finished_ = true;
}

#endif

// template <class G>
// RCLCPP_COMPONENTS_REGISTER_NODE(Simulation<G>)
