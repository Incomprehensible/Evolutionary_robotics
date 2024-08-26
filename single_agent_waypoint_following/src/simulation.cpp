// #include <random>
// #include "single_agent_waypoint_following/simulation.hpp"

// using namespace std::chrono_literals;
// using namespace std::placeholders;

// template <typename G>
// Simulation<G>::Simulation(const rclcpp::NodeOptions & options, const std::string & node_name)
// : Node(node_name, options), tf_buffer_(std::make_shared<rclcpp::Clock>()), tf_listener_(tf_buffer_),
//     evolver_(P_, R*4, {1.0, 5.0})
// {
//     // TMP
//     this->runs_ = S;
//     this->radius_ = R;
//     this->desired_fitness_ = F;

//     this->finished_ = false;
//     points_.resize(S);
//     current_evaluation_ = Stats();

//     evolver_.init_population();

//     this->action_client_ = rclcpp_action::create_client<Setpoint>(this, "go_to_setpoint");
//     this->speed_client_ = this->create_client<agent_interface::srv::SetSpeed>("set_speed");
//     this->config_client_ = this->create_client<agent_interface::srv::SetConfig>("set_config");

//     timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//     this->timer_ = this->create_wall_timer(150ms, std::bind(&Simulation::simulation_run, this), timer_cb_group);
// }

// template <typename G>
// double Simulation<G>::get_fitness()
// {
//     double f = 0;
//     double fail_penalty = 10.0; // TMP
//     double time_penalty = 0.15; // TMP
//     double dist_penalty = 10.0; // TMP
//     double jerk_penalty = 1.5;
//     // double travel_penalty = 10.0;

//     for (const auto& stat : points_) {
//         f += dist_penalty * stat.distance_to_goal;
//         f += time_penalty * stat.total_cycles;
//         f += stat.total_rotations / 2*M_PI;
//         f += stat.distance_traveled / radius_; // for non-equidistant setpoints need to normalize!
//         double jerk = stat.total_jerk / 1000.0;
//         f += (jerk > 3.0)? jerk * jerk_penalty : jerk;
//         f += stat.reached? 0 : fail_penalty;
//         RCLCPP_INFO(this->get_logger(), "dist_to_goal:{%f}, total_cycles:{%d}, distance_traveled:{%f}\ntotal_rotations:{%f}, total_jerk:{%f}, reached:{%d}", stat.distance_to_goal, stat.total_cycles, stat.distance_traveled, stat.total_rotations, stat.total_jerk, stat.reached);
//     }
//     f /= points_.size();
//     return 1 / f;
// }

// template <typename G>
// std::vector<geometry_msgs::msg::Point> Simulation<G>::generate_setpoints()
// {
//     std::vector<geometry_msgs::msg::Point> p;
//     p.resize(runs_);

//     for (size_t i=0; i<runs_; ++i) {
//         p[i] = generate_setpoint(radius_);
//     }

//     return p;
// }

// template <typename G>
// std::vector<geometry_msgs::msg::Point> Simulation<G>::generate_equidistant_setpoints()
// {
//     std::vector<geometry_msgs::msg::Point> p;
//     p.resize(runs_);

//     for (size_t i=0; i<runs_; ++i) {
//         p[i] = generate_equidistant_setpoint(radius_);
//     }

//     return p;
// }

// template <typename G>
// geometry_msgs::msg::TransformStamped::SharedPtr Simulation<G>::get_position()
// {
//     geometry_msgs::msg::TransformStamped odom2robot;

//     try {
//         odom2robot = tf_buffer_.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
//     } catch (tf2::TransformException& e) {
//         RCLCPP_ERROR(this->get_logger(), "Odom to robot transform not found: %s", e.what());
//         return nullptr;
//     }
//     return std::make_shared<geometry_msgs::msg::TransformStamped>(std::move(odom2robot));
// }

// template <typename G>
// double Simulation<G>::get_distance(geometry_msgs::msg::Point& p1, geometry_msgs::msg::Point& p2)
// {
//     return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
// }

// // double generateRandom(double radius) {
// //     // Generate a uniform random number between 0 and 1
// //     std::random_device seed; // obtain a random number from hardware

// //     // distribution over closed interval
// //     std::uniform_real_distribution d(0.0, radius);
// //     auto gen = std::bind(d, std::mt19937(seed()));
    
// //     return gen();
// // }

// template <typename G>
// void Simulation<G>::simulation_run()
// {
//     this->timer_->cancel();
//     rclcpp::Rate rate(1s);
//     double speed = 2; // TMP
//     double allowance_time = 4.0; // TMP
    
//     std::vector<PGenome>& population = evolver_.get_population();
//     // std::vector<Genome<double, 2>>& population = evolver_.get_population();
//     assert(population.size() == P_);

//     auto setpoints = generate_equidistant_setpoints();

//     auto speed_req = std::make_shared<agent_interface::srv::SetSpeed::Request>();
//     speed_req->speed = speed;
//     while (!speed_client_->wait_for_service(1s)) {
//         if (!rclcpp::ok()) {
//             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the speed service. Exiting.");
//             rclcpp::shutdown();
//         }
//         RCLCPP_INFO(this->get_logger(), "Speed service not available, waiting again...");
//     }
//     speed_client_->async_send_request(speed_req);

//     auto request = std::make_shared<agent_interface::srv::SetConfig::Request>();

//     for (size_t n=0; n<P_; ++n) {
//         PGenome& p = population[n];
//         // Genome<double, 2>& p = population[n];
    
//         request->k_l = p.genes_[0].raw_value;
//         request->k_ha = p.genes_[1].raw_value;
//         while (!config_client_->wait_for_service(1s)) {
//             if (!rclcpp::ok()) {
//                 RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the configuration service. Exiting.");
//                 rclcpp::shutdown();
//             }
//             RCLCPP_INFO(this->get_logger(), "Configuration service not available, waiting again...");
//         }
//         config_client_->async_send_request(request);

//         for (size_t i=0; i<runs_; ++i) {
//             finished_ = false;

//             if (!this->action_client_->wait_for_action_server()) {
//                 RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//                 rclcpp::shutdown();
//             }

//             geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
//             while ((odom2robot_ptr = get_position()) == nullptr)
//                 rate.sleep();
//             auto odom2robot = *(odom2robot_ptr.get());
//             geometry_msgs::msg::Point pose;
//             pose.x = odom2robot.transform.translation.x;
//             pose.y = odom2robot.transform.translation.y;
//             pose.z = 0;
    
//             auto goal_msg = Setpoint::Goal();
//             goal_msg.setpoint.x = setpoints[i].x + pose.x;
//             goal_msg.setpoint.y = setpoints[i].y + pose.y;
//             goal_msg.setpoint.z = setpoints[i].z + pose.z;
//             // goal_msg.setpoint = generate_setpoint();

//             double dist = get_distance(goal_msg.setpoint, pose);

//             double timeout = dist / speed;
//             auto timeout_duration = std::chrono::duration<double>(timeout + allowance_time);

//             RCLCPP_DEBUG(this->get_logger(), "Sending goal...");

//             auto send_goal_options = rclcpp_action::Client<Setpoint>::SendGoalOptions();
//             send_goal_options.goal_response_callback = std::bind(&Simulation::goal_response_callback, this, _1);
//             send_goal_options.feedback_callback = std::bind(&Simulation::feedback_callback, this, _1, _2);
//             send_goal_options.result_callback = std::bind(&Simulation::result_callback, this, _1);
//             auto goal_handle_future = this->action_client_->async_send_goal(goal_msg, send_goal_options);

//             // Wait for the goal to be accepted or rejected
//             // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
//             // {
//             //     RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
//             //     continue;
//             // }
//             auto goal_handle = goal_handle_future.get();
//             if (!goal_handle)
//             {
//                 RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
//                 continue;
//             }

//             auto start_time = std::chrono::steady_clock::now();

//             while (!finished_) {
//                 auto current_time = std::chrono::steady_clock::now();
//                 if (current_time - start_time >= timeout_duration)
//                 {
//                     RCLCPP_DEBUG(this->get_logger(), "Goal timeout! Canceling goal...");
//                     auto cancel_future = this->action_client_->async_cancel_goal(goal_handle);
//                     // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
//                     // {
//                     //     RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
//                     //     rclcpp::shutdown();
//                     // }
//                     auto cancel_response = cancel_future.get();
//                     if (cancel_response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
//                         RCLCPP_DEBUG(this->get_logger(), "Goal successfully canceled");
//                     else
//                         RCLCPP_WARN(this->get_logger(), "Goal failed to cancel");

//                     break;
//                 }
//                 RCLCPP_DEBUG(this->get_logger(), "Waiting...:");
//                 rate.sleep();
//             }
//             while (!finished_) rate.sleep();

//             if (!current_evaluation_.reached) {
//                 // define penalty for goal failure
//                 RCLCPP_DEBUG(this->get_logger(), "Goal fail!");
//             }

//             points_[i] = current_evaluation_;
//             // points_.push_back(current_evaluation_);
//         }
//         double fitness = get_fitness();
//         p.fitness = fitness;
//         RCLCPP_INFO(this->get_logger(), "Candidate: K_l: {%f}, K_ha: {%f}, Fitness: {%f}", request->k_l, request->k_ha, fitness);
//         // points_.clear(); // tmp
//     }
    
//     double best_fitness = evolver_.rank_candidates();
//     RCLCPP_INFO(this->get_logger(), "Best Fitness: {%f}", best_fitness);

//     if (best_fitness < desired_fitness_)
//     {
//         RCLCPP_INFO(this->get_logger(), "Generating new population...");
//         evolver_.generate_new_population();
//         return this->timer_->reset();
//     }
// }

// template <typename G>
// void Simulation<G>::goal_response_callback(std::shared_ptr<GoalHandleSetpoint> future)
// {
//     auto goal_handle = future.get();
//     if (!goal_handle) {
//         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//         RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
// }

// template <typename G>
// void Simulation<G>::feedback_callback(GoalHandleSetpoint::SharedPtr, const std::shared_ptr<const Setpoint::Feedback> feedback)
// { 
//     RCLCPP_DEBUG(this->get_logger(), "reached: {x:%f, y:%f}", feedback->current_pose.position.x, feedback->current_pose.position.y);
// }

// template <typename G>
// void Simulation<G>::result_callback(const GoalHandleSetpoint::WrappedResult & result)
// {
//     switch (result.code) {
//         case rclcpp_action::ResultCode::SUCCEEDED:
//             RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
//             break;
//         case rclcpp_action::ResultCode::ABORTED:
//             RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//             return;
//         case rclcpp_action::ResultCode::CANCELED:
//             RCLCPP_INFO(this->get_logger(), "Goal was canceled");
//             break;
//         default:
//             RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//             return;
//     }

//     current_evaluation_ = result.result->evaluation;
//     RCLCPP_DEBUG(this->get_logger(), "Final pose reached: {x:%f, y:%f}", result.result->pose.position.x, result.result->pose.position.y);
//     finished_ = true;
// }

// // void Simulation::simulation_run()
// // {
// //     rclcpp::Rate rate(1s);

// //     for (size_t i=0; i<runs_; ++i) {
// //         Setpoint p = generate_setpoint();
// //         controller_->set_goal(p);
// //         controller_->enable();

// //         while ((controller_->get_goal_status()) != PIDController::FINISHED) {
// //             RCLCPP_DEBUG(this->get_logger(), "Waiting...:");
// //             rate.sleep();
// //         }

// //         // define and use penalty after checking GoalResult
// //         points_.push_back(controller_->get_performance());
// //         controller_->reset();
// //     }
// //     RCLCPP_DEBUG(this->get_logger(), "Fitness: {%f}", get_fitness());
// // }

// // geometry_msgs::msg::Point Simulation::generate_setpoint()
// // {
// //     rclcpp::Rate rate(1s);

// //     geometry_msgs::msg::TransformStamped::SharedPtr odom2robot_ptr;
// //     // geometry_msgs::msg::Pose::SharedPtr odom2robot_ptr;
// //     // simulated robot is not spawned yet - wait
// //     while ((odom2robot_ptr = get_position()) == nullptr) {
// //         rate.sleep();
// //     }

// //     auto odom2robot = *(odom2robot_ptr.get());

// //     double x_origin = odom2robot.transform.translation.x;
// //     double y_origin = odom2robot.transform.translation.y;
// //     // double x_origin = odom2robot.position.x;
// //     // double y_origin = odom2robot.position.y;

// //     tf2::Quaternion orientation;

// //     // for generating random orientation - not needed now
// //     // orientation.setX(odom2robot.orientation.x);
// //     // orientation.setY(odom2robot.orientation.y);
// //     // orientation.setZ(odom2robot.orientation.z);
// //     // orientation.setW(odom2robot.orientation.w);

// //     orientation.setX(odom2robot.transform.rotation.x);
// //     orientation.setY(odom2robot.transform.rotation.y);
// //     orientation.setZ(odom2robot.transform.rotation.z);
// //     orientation.setW(odom2robot.transform.rotation.w);
    
// //     double yaw_origin = tf2::getYaw(orientation);

// //     // randomly generate dx, dy, dyaw
// //     double dx = generateEdgeBiasedRandom(radius_);
// //     double dy = generateEdgeBiasedRandom(radius_);

// //     geometry_msgs::msg::Point goal;
// //     goal.x = x_origin + dx;
// //     goal.y = y_origin + dy;
// //     goal.z = 0;
// //     return goal;
// // }

// RCLCPP_COMPONENTS_REGISTER_NODE(Simulation)