# Evolutionary_robotics

ROS2 framework written in C++ for applying genetic algorithm to enable mobile robot controllers evolution.
It utilizes the abstractions and polymorphism provided by C++ for extendability to different algorithms, controller types and use-cases. 
However, the implementation is not finalized yet, and user needs to manually specify the classes/interfaces in some cases.
This framework uses TurtleBot3 modified model and Gazebo Classic simulation.

## How it works
Each session consists of a population of robot controllers. Each controller specification consists of a Genome and Phenotype. Genome is evolved using `Evolver` class and is controlled by `Simulation` client class. Phenotype is transferred to `Controller` action server node which implements the actual robot steering. `Controller` then sends the statistics and performance evaluation of each candidate back to `Simulation` client which calculates the fitness of the candidate.
Fitness function is still in an experimental phase but produces stable results.
Simulation may require hunderds of epochs to evolve the controllers, especially the CNN-controllers. Which is why the accelerated simulation environment is utilized. The TurtleBot3 model used was modified so that unused sensors are not rendered.
P-controllers should converge to an optimal solution after tens of epochs. Theoretically any speed can be specified. Practically the algorithm was tested on speeds up to `2.5 m/s`.
It would be faster to run the simulation without visualization but the user might want to obseve how the robot moves in 3D.

## Genetic algorithm implementations
* P-controller evolution is based on my implementation [here](https://github.com/Incomprehensible/Mobile_robot_programming) and evolves the P-gains.
* CNN-controller evolution evolves the weights of the neural network controlling the robots.
Currently such operations as Elitism, Reproduction, probabolistic Crossover and Mutation are implemented.

## Project structure
Project contains two packages:
* agent_interface: defines `speed interface` for setting the maximum allowed speed for the simulated robot and `agent specification interfaces`
* single_agent_waypoint_following: implements a usage case which demostrates the genetic algorithm functionality

## Interfaces
* `speed interface` is implemented as a service. Currently the `Simulation` client node sends a service request to setup the speed for the controllers population.
* `agent interface` files specify the input parameters for configuring the various types of controllers. Currently there are two types of controllers implemented: P-controller and CNN-controller.

## Controllers
### P-controller
P-controllers learn to control the robot with P-gains evolution.
Configuration description for the P-controller:
```python
# P-controller settings
float64 k_l
float64 k_ha
```

## Goal specification
Current implementation of the basic usage case evolves the mobile robot controllers to steer to the goal.
Communication between `Simulation` client and `Controller` worker server uses the action interface:
```python
# Define the goal
geometry_msgs/Point setpoint
---
# Define the result
Stats evaluation
geometry_msgs/Pose pose 
---
# Define the feedback
geometry_msgs/PoseStamped current_pose
```

### CNN-controller
This controller uses a Convolutional Neural Network. Its genome is defined as a fixed topology neural network containing 3 hidden layers by 20 neurons. Two output neurons use sigmoid for linear speed and tahn for angular speed outputs. This topology and implementation can be customized, although there's no convenient way of doing it yet (you need to change it in the code).
Genetic algorithm evolves the weights of the neural network which is encoded as a genome and expressed as a phenotype inside the `Controller` node. The phenotype is sent to the `Controller` as a configuration.
Configuration description for the CNN-controller:
```python
# CNN-controller config
uint32 num_inputs
uint32 num_outputs
uint32 num_hidden
uint32 neurons_per_hidden
float64[] weights
```

Below is the video demostration of the process. The rviz2 visualization uses robot's base_link frame as center of coordinates. White square represents the robot while blue dot is a goal from the randomly generated set.
[![Watch how CNN-controllers evolve](https://img.youtube.com/vi/TGR7oGfYvO4/maxresdefault.jpg)](https://youtu.be/TGR7oGfYvO4)

## How to build
Run from the project root directory:
```zsh
agent_interface
$ cd Evolutionary_robotics
$ colcon build && source install/setup.bash
```

## How to run
Overwrite environment variables:
```zsh
$ export TURTLEBOT3_MODEL=waffle
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/model
```

## Further steps
* Add visualization of graphs and statistics
* Add extra functionality to export and load the previous session of controllers to continue the evolution using `Google Protobuf`.
* Finally adding [NEAT](https://en.wikipedia.org/wiki/Neuroevolution_of_augmenting_topologies) and rt-NEAT algorithms which were the main driven force behind the creation of this project.
* Implement use case of collision-free navigation using `Navigation2` framework.

## Current issues
* Epoch numbering is improperly implemented and needs to be fix to enable sessions.
* Small bug in CNN-controller implementation messes up memory when even number of candidates is used.

This code is part of my Mobile Robot Programming experiments and shouldn't be used in real-world hardware applications.

