# assignment2_rt_part1
**Sarvenaz Ashoori**  
**ID: 6878764**

**ros assignment2_rt_part1**
## Introduction

This project creates a ROS-based system to control a robot within a Gazebo simulation. It highlights the use of Action Clients and Service Nodes to dynamically set and retrieve target coordinates. The simulation allows user interaction and demonstrates how nodes communicate via ROS.

The repository contains a ROS package with the following components:
 - **Action Client Node**: Manages target setting and communicates with the action server.
 - **Service Node**: Offers a service to retrieve the most recent target coordinates.
 - **Launch File**: Initiates all the nodes.

 ## Node and Launch File Overview

### Action Client Node

The **Action Client Node** facilitates interaction with the Action Server in a client-server model. This node performs the following tasks:
 - Sends target coordinates to the action server.
 - Handles user input:
    - Prompts the user to input the `x` and `y` coordinates for the robot's destination.
    - Allows the user to cancel the goal while the robot is in motion.
 - Subscribes to the `/odom` topic.
 - Publishes the robot's current position and velocity to a custom ROS topic.

Key Features:
 - Receives feedback and monitors the goal status from the action server.
 - Dynamically updates the target coordinates based on user input.

### Service Node

This node provides a service that:
 - Retrieves the last set target coordinates provided by the user.

Key Features:
 - Listens for service requests on `/get_last_target` and responds with the most recent target coordinates.
 - Ensures consistency with the Action Client Node by utilizing shared parameters.
 ### Folder and File Structure

- **`/launch`**: Contains the launch files.
    - `coordinate_control_py.launch`: Launches both nodes simultaneously for the Python version.

- **`/msg`**: Holds custom message definitions.
    - `robot_status.msg`: Defines a custom message to publish the robot's position and velocity.

- **`/scripts`**: Includes Python scripts for the nodes.
    - `action_client_node.py`: Python script for the action client node.
    - `service_node.py`: Python script for the service node.

- **`/srv`**: Contains custom service definitions.
    - `get_last_target.srv`: Defines the service for retrieving the last set target coordinates.

- **`/CMakeLists.txt`**: Specifies the build rules and dependencies for the package.

- **`/package.xml`**: Lists the package dependencies and metadata.

- **`/README.md`**: This file (Documentation).

## Getting Started (Please Read First)

### Prerequisites
Ensure that **`ROS Noetic`** is installed on your system before proceeding. <br>
If ROS is not set up yet, follow the official installation guide for ROS Noetic on Ubuntu: <br>
(https://wiki.ros.org/noetic/Installation/Ubuntu) <br>

Additionally, you’ll need **`Python 3`** and the **`python-is-python3`** package to run this project. Make sure these dependencies are installed. If not, you can install them by running the following commands:
```bash
sudo apt-get update
sudo apt-get install python3
sudo apt-get install python-is-python3
```
After installing the necessary dependencies, you can proceed with setting up your ROS workspace.
### Setup 

#### 1. Create your ROS workspace
Create a new workspace (or use an existing one) and navigate to the `src` directory:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

#### 2. Clone the `assignment_2_2024` package
Before cloning this repository, you need to obtain the `assignment_2_2024` package, which is responsible for initializing the simulation environment and the action server. Clone the `assignment_2_2024` repository into the `src` folder of your workspace:
```bash
git clone https://github.com/CarmineD8/assignment_2_2024.git
```

#### 3. Clone this repository
Once the `assignment_2_2024` package is cloned, clone this repository into the `src` folder of your workspace:
```bash
git clone https://github.com/Rubin-unige/assignment2_rt_part1.git
```

#### 4. Build the workspace
After both repositories are cloned, return to the root directory of your workspace and build the packages using `catkin_make`:
```bash
cd ~/ros_ws
catkin_make
```

#### 5. Add the Workspace to your ROS Environment
To ensure that your workspace is sourced automatically when you open a new terminal session, add the following to your `.bashrc` file:
```bash
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 6. Source the workspace
After building and sourcing, source the workspace manually for the first time in your current terminal session:
```bash
source ~/ros_ws/devel/setup.bash
```#### 1. Launch the `assignment_2_2024` Package

Before starting your nodes, ensure that the simulation environment is active. To launch the simulation in Gazebo and Rviz, execute the following command:
```bash
roslaunch assignment_2_2024 assignment1.launch
```
This will initialize the robot in the simulation and start the action server, which will interact with the Action Client Node. <br>
**Note**: Wait until everything has fully loaded before proceeding.

#### 2. Launch the Action Client Node and Service Node

Once the simulation environment is up and running, you can launch either the **C++** or **Python** version of the nodes. 
```
#### Running the Python Version
To launch the Python version, use this command:
```bash
roslaunch assignment2_rt_part1 assignment2_part1.launch
```
This will start both the Action Client Node and the Service Node at the same time.

#### 3. Running the ROS Service to Retrieve the Last Coordinates
After the Action Client Node has started, you can fetch the most recent target coordinates by calling the following service:
```bash
rosservice call /last_target
```
This will return the last target coordinates set for the robot based on the latest user input.

#### 4. Stopping the Nodes

To stop the nodes, simply press `Ctrl+C` in the terminal where each node is running. This will terminate the nodes and halt the simulation.


**Summary**
This project implements a ROS-based system for controlling a robot in a Gazebo simulation, using Action Clients and Service Nodes for target coordinate management. The system allows users to set target positions for the robot, retrieve the last target coordinates, and cancel or update goals dynamically.

Key components:
1. **Action Client Node**: Interacts with the Action Server to send target coordinates, handles user input for goal setting, subscribes  for robot position, and publishes the robot's status.
2. **Service Node**: Provides a service to fetch the last target coordinates.
3. **Launch File**: Starts both nodes simultaneously.

The project includes a folder structure with launch files, custom message and service definitions, Python scripts for the nodes, and configuration files for building and managing the package. 

To get started, ensure **ROS Noetic** and **Python 3** are installed, and set up a ROS workspace. Clone both the `assignment_2_2024` package (for the simulation and action server) and this repository, build the workspace, and source it. 

To run the system, first launch the simulation environment with `roslaunch assignment_2_2024 assignment1.launch`, then launch either the C++ or Python version of the nodes. After the nodes are running, you can call the service to retrieve the last target coordinates. To stop the nodes, press `Ctrl+C` in the terminal. 

The communication flow between the Action Client Node and Action Server allows the robot to navigate to specified goals, cancel ongoing tasks, or confirm when a goal is reached, ensuring interactive control over the robot’s movement.
