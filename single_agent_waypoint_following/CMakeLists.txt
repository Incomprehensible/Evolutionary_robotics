cmake_minimum_required(VERSION 3.8)
project(single_agent_waypoint_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(agent_interface REQUIRED)

cmake_minimum_required(VERSION 3.8)
project(single_agent_waypoint_following)

# Add compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(agent_interface REQUIRED)

# Include directories
include_directories(include)

# target_include_directories(simulation_component PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_include_directories(controller_component PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

add_executable(NEAT_simulator
    src/simulation.cpp
    src/main_sim.cpp)

ament_target_dependencies(NEAT_simulator
    "rclcpp"
    "rclcpp_components"
    "rcl_interfaces"
    "std_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    "nav2_msgs"
    "agent_interface"
)

add_executable(NEAT_controller
    src/controller.cpp
    src/PID_controller.cpp
    src/CNN.cpp
    src/CNN_controller.cpp
    src/main_controller.cpp)

ament_target_dependencies(NEAT_controller
    "rclcpp"
    "rclcpp_components"
    "rcl_interfaces"
    "std_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    "nav2_msgs"
    "angles"
    "agent_interface"
)

install(TARGETS
    NEAT_simulator
    NEAT_controller
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
