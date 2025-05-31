# Custom DWA Controller Plugin for Nav2

This repository provides a custom Dynamic Window Approach (DWA) controller plugin for the ROS 2 Navigation Stack (Nav2). It enhances path tracking, obstacle avoidance, and velocity control for mobile robots such as TurtleBot3. This repository is tested in ROS2 Humble with [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3.git) Robot in Gazebo Classic Simulator.

---

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- TurtleBot3 packages
- Nav2 Stack
- Git
- Gazebo Classic

---

## Prerequisites:

Follow the official TurtleBot3 Quick Start Guide:  
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

Install Simulation packages following this link:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation

Basic setup:

```bash
# Set the TurtleBot3 model (e.g., burger)
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Source ROS 2
source /opt/ros/humble/setup.bash
```

---
## Clone This Repository

Navigate to your TurtleBot3 workspace and clone this repository inside the ```src``` folder:
```bash
cd ~/turtlebot3_ws/src

# Clone the custom DWA controller plugin repo
git clone https://github.com/tirthvyas7/custom_dwa_plugin_nav2.git

# Build the workspace
cd ~/turtlebot3_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```
## Changing paramater file and updating parameters:
Copy the file at: https://github.com/tirthvyas7/custom_dwa_plugin_nav2/blob/main/custom_dwa_controller/src/burger.yaml and paste it in the ```turtlebot3/turtlebot3_navigation2/param/humble``` location in your ```turtlebot3_ws```.

Then you can update the following paramaters in the paramater file.
```bash
FollowPath:
      plugin: "custom_dwa_controller::DWALocalPlanner"
      max_vel_x: 0.3 # Maximum allowed linear velocity
      min_vel_x: 0.0 # Minimum allowed linear velocity
      acc_lim_x: 1.0 # Maximum allowed linear acceleration/deacceleration
      acc_lim_theta: 5.0 # Maximum allowed angular acceleration/deacceleration
      max_vel_theta: 3.0  # Maximum allowed angular velocity(in both directions)
      sim_time: 1.5 # Time to simulate ahead 
      vx_samples: 60 # Number of linear velocity samples
      vtheta_samples: 40 # Number of angular velocity samples
      goal_weight: 10000.0 # Reward for following final goal
      obstacle_weight: 5000.0 # Reward for avoiding obstacles
      heading_weight: 1000000.0 # Reward for maintaining heading towards the global path
      velocity_weight: 10000.0 #Reward for selecting higher velocities

```

## Building the code:
Open a fresh terminal and run these commands:

```bash
cd ~/turtlebot3_ws
colcon build
```

## Running the Launch File:
Run these commands in the terminal where you buit the code:
```bash
# Ensure you are in turtlebot3_ws
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In a fresh terminal run below commands for running the navigation stack with our custom plugin:

```bash
source install/setup.bash
#Check that the paramater file is replaced at location turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml and provide map file location correctly
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/custom_dwa_plugin_nav2/maps/map.yaml
```

You can visualise the selected trajectorie in Rviz2 by selecting ```/best_trajectory_path``` topic in the Local Plan Section and all the sampled trajectories by selecting the topic ```\all_trajectories``` in Trajectories section in Controller Server. 


