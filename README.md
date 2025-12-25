Learning-based Navigation for a Mobile Robot in Simulation using ROS2 and Reinforcement Learning

This project implements a learning-based navigation framework for a mobile robot using ROS 2 (Jazzy), Gazebo, and Reinforcement Learning.
A TurtleBot3 robot learns to navigate in simulation using LiDAR perception as input and continuous velocity commands as actions, forming a complete MDP-based control pipeline suitable for academic research and continuation into real-world transfer.

🎯 Project Objectives

This project aims to:

Develop a ROS 2 navigation environment compatible with RL frameworks.

Integrate Gazebo simulation, LiDAR sensing, and odometry feedback.

Formulate state, action, reward functions under a clear MDP definition.

Train learning-based navigation policies using PPO (Stable-Baselines3).

Provide a reproducible framework suitable for MSc-level research work and expansion to:

obstacle avoidance benchmarks

goal-directed navigation

sim-to-real transfer

🧠 Problem Definition (Scientific Perspective)

Robot navigation is modeled as a Markov Decision Process (MDP):

State (Observation):

Downsampled LiDAR scan

Distance-to-goal

Heading feature

Action Space (Continuous):

a = [linear_velocity, angular_velocity]


Reward Function:

Penalize large distance to goal

Heavy penalty near collision

Positive termination on success

Termination Conditions:

Reaching the goal

Collision with obstacles

This structure allows comparative evaluation against classical navigation and enables RL algorithm experimentation.

🏗 System Architecture
ROS 2 Jazzy
      │
      ├── TurtleBot3 in Gazebo
      │      ├── /scan (LiDAR)
      │      ├── /odom (Odometry)
      │      └── /cmd_vel (Velocity Commands)
      │
      └── RL Environment (Gym-compatible)
             │
             ├── PPO Agent (Stable-Baselines3)
             └── Training & Evaluation Pipelines

🚀 Features Implemented

✔ ROS 2 package (rl_nav)
✔ Baseline random navigation controller
✔ Gym-compatible Reinforcement Learning environment
✔ Continuous action state space
✔ Reward shaping & termination conditions
✔ Gazebo integration
✔ PPO training pipeline
✔ MSc-level structure & reproducibility

🛠 Installation & Setup
Requirements

Ubuntu 24.04

ROS 2 Jazzy

Gazebo

Python 3.12

TurtleBot3 packages

▶️ Running the Simulation
1️⃣ Launch TurtleBot3 in Gazebo
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

🤖 Reinforcement Learning Environment

The environment is implemented in:

rl_nav/rl_nav/env.py


It exposes:

reset()

step()

observation_space

action_space

Fully compatible with Stable-Baselines3 workflows.

🧪 RL Training (PPO)

Activate RL virtual environment:

source ~/rl_venv/bin/activate


Run training:

python3 train_ppo.py


Model output:

models_ppo/ppo_turtlebot.zip

📊 Evaluation & Future Work

This framework is designed for MSc-level extensions such as:

Quantitative performance evaluation

Success rate & collision metrics

Energy efficiency

Domain Randomization

Curriculum learning

Transfer learning to real robot

Potential research directions:

Comparison with classical planners

Comparison of PPO vs SAC vs TD3

Multi-goal navigation

Dynamic environments

🎓 Academic Relevance

This project demonstrates:

Strong competence in robotics simulation

Ability to design learning-based control systems

ROS 2 development proficiency

Understanding of MDP formulation & RL training pipelines

Research mindset and reproducibility awareness

Suitable for:

MSc dissertation foundation

PhD proposal support

Research lab applications

Robotics industry portfolios

👤 Author

Panagiota Grosdouli

📜 License

Open for academic and research use.
