# AMR Fleet Simulation — ROS 2 & Gazebo

A robotics simulation project focused on **Autonomous Mobile Robots (AMRs)** in a warehouse environment, built using ROS 2 Jazzy and Gazebo Sim. The project establishes the foundation for LiDAR-based obstacle detection, autonomous navigation, and future multi-robot fleet coordination.

## Project Overview

Warehouse robots must navigate shared spaces, detect nearby obstacles, and move safely around storage racks and other objects. This project develops a simulated AMR environment to explore these capabilities before moving to physical hardware.

The current implementation includes a warehouse world, a differential-drive robot, LiDAR sensing, odometry, and an initial ROS 2 obstacle-detection node.

## Features

- **Warehouse simulation:** A Gazebo environment with a floor and storage racks arranged around robot aisles.
- **Differential-drive control:** Move the robot using ROS 2 velocity commands.
- **LiDAR sensing:** Simulated 360° laser scanning for observing the surrounding environment.
- **ROS 2 integration:** Communication between Gazebo and ROS 2 through `ros_gz_bridge`.
- **Odometry:** Access to the robot's simulated odometry data.
- **Obstacle detection:** An initial Python node processes LiDAR scans and reports the nearest valid measurement within the front sector.
- **RViz2 visualization:** Visualize LiDAR scan points and inspect robot sensor data.

## Technology Stack

| Technology | Purpose |
|---|---|
| C++ / Python | Robotics development and ROS 2 nodes |
| ROS 2 Jazzy | Robot middleware and communication |
| Gazebo Sim | Physics-based robot simulation |
| URDF | Robot description |
| SDF | Warehouse world description |
| `ros_gz_bridge` | Gazebo–ROS 2 message integration |
| RViz2 | Sensor-data visualization |
| Git & GitHub | Version control and project documentation |

## System Architecture

```text
             Gazebo Warehouse
                    |
             Simulated AMR
                    |
        +-----------+-----------+
        |           |           |
      LiDAR      Odometry    /cmd_vel
        |           |           ^
        v           v           |
   /lidar      Robot motion     |
        |                       |
        v                       |
  ROS 2 Bridge                  |
        |                       |
        v                       |
 LiDAR Obstacle Detector        |
        |                       |
        v                       |
 Front-clearance information    |
                                |
                   Velocity control
                   (manual testing)
```

The obstacle detector currently reports nearby obstacles; automatic stopping and autonomous navigation are future development steps.

## Repository Structure

```text
amr-fleet-sim/
├── src/
│   ├── amr_description/
│   │   ├── launch/
│   │   │   └── sim.launch.py
│   │   ├── urdf/
│   │   │   └── amr.urdf
│   │   ├── worlds/
│   │   │   └── warehouse.sdf
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── amr_obstacle_detection/
│       ├── amr_obstacle_detection/
│       │   └── obstacle_detector.py
│       ├── package.xml
│       └── setup.py
│
├── Dockerfile
├── PROGRESS.md
├── .gitignore
└── README.md
```

## Prerequisites

- Ubuntu 24.04, or WSL2 running Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim compatible with the installed ROS–Gazebo integration
- `ros_gz_sim` and `ros_gz_bridge`
- RViz2 for optional visualization

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/Robo-Synaptic-Vaish/amr-fleet-sim.git
cd amr-fleet-sim
```

### 2. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build the workspace

```bash
colcon build
source install/setup.bash
```

### 4. Launch the warehouse simulation

In Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/reena/amr-fleet-sim
source install/setup.bash
ros2 launch amr_description sim.launch.py
```

If you cloned the project elsewhere, replace the workspace path with your actual directory.

### 5. Spawn the AMR

In Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/reena/amr-fleet-sim
source install/setup.bash

ros2 run ros_gz_sim create \
  -topic robot_description \
  -name amr
```

### 6. Bridge LiDAR data to ROS 2

In Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```

Verify that the topic exists:

```bash
ros2 topic info /lidar
ros2 topic echo /lidar --once
```

### 7. Run the obstacle detector

In Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/reena/amr-fleet-sim
source install/setup.bash

ros2 run amr_obstacle_detection obstacle_detector
```

The node examines LiDAR measurements within approximately ±30° of the front direction and reports front clearance. Its default warning threshold is 1.0 metre.

**Note:** The detector currently provides information and warnings only. It does not automatically stop the robot.

## Example Robot Control

To move the robot forward, publish velocity commands:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Stop the publisher with `Ctrl+C`, then send a zero-velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

These commands are for simulation testing and manual control, not a safety-rated motion controller.

## Development Roadmap

- [x] Set up the ROS 2 Jazzy workspace
- [x] Create the differential-drive AMR model
- [x] Build the warehouse world in Gazebo
- [x] Test velocity-based robot movement
- [x] Integrate simulated LiDAR data with ROS 2
- [x] Create the initial LiDAR obstacle-detection package
- [ ] Verify detector output against live simulation scans
- [ ] Publish structured obstacle information
- [ ] Implement obstacle-triggered stopping
- [ ] Integrate localization and Nav2 navigation
- [ ] Add multiple AMRs to the warehouse
- [ ] Explore task allocation, collision avoidance, and fleet coordination

## Learning Objectives

This project is intended to develop practical understanding of:

- Robot description and simulation environments
- ROS 2 nodes, topics, and message types
- Sensor-data processing
- Differential-drive motion and odometry
- Robotics debugging and visualization
- The transition from individual robot control to multi-robot coordination

## Future Scope

The long-term goal is to extend this simulation into a multi-AMR warehouse system with autonomous navigation, obstacle-aware planning, and coordinated task execution. The simulation-first approach makes it possible to test algorithms before deploying them on physical robots.

## Author

**Vaishnavi Singh**

B.Tech — Computer Science and Engineering (Robotics & AI)

GitHub: [Robo-Synaptic-Vaish](https://github.com/Robo-Synaptic-Vaish)

---

*This project is under active development. Features listed in the development roadmap are not necessarily implemented or validated yet.*
