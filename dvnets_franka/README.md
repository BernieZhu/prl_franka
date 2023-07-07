# Moveit Planning

This repository contains code for performing planning and control using MoveIt and Franka Panda robot. It provides a set of ROS nodes and services for controlling the robot arm and gripper.

## Installation

### Dependencies

Make sure you have the following dependencies installed:

- ROS
- MoveIt
- [Franka Emika Panda MoveIt Config Package](https://github.com/ros-planning/panda_moveit_config)
- Python

To install the ROS packages `dvnets_franka`, `dvnets_franka_msgs`, and `panda_moveit_config`, you can follow the steps below:

1. Make sure you have ROS installed on your system. You can refer to the ROS installation guide for instructions specific to your operating system: [ROS Installation Guide](http://wiki.ros.org/ROS/Installation)

2. Create a catkin workspace if you don't have one already. Open a terminal and run the following commands:

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

3. Clone the repositories for the required packages `dvnets_franka`, `dvnets_franka_msgs`, and `panda_moveit_config` into the `src` directory of your catkin workspace:

4. Install any additional dependencies required by the packages. You can refer to the package documentation or `package.xml` files for information on dependencies. Use the `rosdep` tool to install the dependencies:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the packages by running `catkin_make`:

   ```bash
   catkin_make
   ```

6. Source the generated setup script to add the packages to your ROS environment:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

   It's recommended to add this line to your `~/.bashrc` file to automatically source the setup script when you open a new terminal.

## Getting Started

1. Activate the PRL environment:

   ```
   activate PRL
   ```

2. Launch the MoveIt planning configuration for the Panda robot with RViz visualization and gripper enabled:

   ```
   roslaunch panda_moveit_config panda_control_moveit_rviz.launch load_gripper:=true robot_ip:="172.16.0.2" launch_rviz:=true
   ```

3. Run the controller server and other ROS nodes:

   ```
   cd dvnets_franka/scripts
   python franka_moveit_controller_server.py
   python pickandplace_demo.py
   ```

## Code Explanation

### franka_moveit_controller_server.py

`franka_moveit_controller_server.py` provides a controller server for the Franka Panda robot. It initializes MoveIt and sets up the necessary interfaces for planning and control. It also defines the `FrankaController` class, which contains methods for moving the robot arm and controlling the gripper. The script subscribes to joint state updates and provides services for moving the robot arm to a target pose, setting the gripper position, and returning the arm to the home position.

The following ROS services are provided by the `franka_moveit_controller_server.py` script:

- `franka_goto_pose`: Moves the robot arm to the specified pose.
- `franka_set_gripper`: Sets the gripper to open or close.
- `franka_home`: Sends the robot arm to the home position.

### pickandplace_demo.py

`pickandplace_demo.py` demonstrates how to perform pick and place operations using the Franka Panda robot. It subscribes to pose_topic to get the pose of the object, and uses the `franka_goto_pose` and `franka_set_gripper` services to control the robot arm and gripper.

## Notes
- The code sets up a planning scene and adds a table object and a octomap from the input point cloud to it. Modify the code to add or remove objects from the scene as required.