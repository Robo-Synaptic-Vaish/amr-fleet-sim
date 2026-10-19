# AMR Warehouse Simulation — Progress Report

Last updated: 20 October 2026

## Project goal
Build a ROS 2 warehouse AMR simulation as a foundation for
obstacle detection, autonomous navigation, and eventually
multi-AMR fleet coordination.

## Environment
- Windows host with WSL2
- Ubuntu 24.04 (Ubuntu-24.04)
- ROS 2 Jazzy
- Gazebo Sim 8
- Workspace: /mnt/c/Users/reena/amr-fleet-sim
- ROS package: amr_description

## Working components
- Gazebo warehouse world with floor and shelf/rack models.
- Differential-drive AMR that responds to /cmd_vel.
- GPU LiDAR configured for a 360-degree scan, 10 Hz, and
  0.1–10.0 m range.
- Gazebo Sensors system enabled in warehouse.sdf.
- LiDAR bridge publishes sensor_msgs/msg/LaserScan on /lidar.
- RViz2 LaserScan display works and shows shelf surfaces as
  red points.
- Gazebo odometry is published on /model/amr/odometry.
- ROS 2 odometry bridge works with nav_msgs/msg/Odometry.
- Movement test confirmed the AMR moves and odometry position
  changes while the LiDAR points move with it.

## Important project files
- src/amr_description/urdf/amr.urdf
- src/amr_description/launch/sim.launch.py
- src/amr_description/worlds/warehouse.sdf
- src/amr_description/CMakeLists.txt
- src/amr_description/package.xml

Backups are also present. Keep them until the current files
have been verified and committed.

## Start the simulation
Terminal 1:
    cd /mnt/c/Users/reena/amr-fleet-sim
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch amr_description sim.launch.py

Terminal 2 — spawn the robot:
    cd /mnt/c/Users/reena/amr-fleet-sim
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run ros_gz_sim create -topic robot_description -name amr

Terminal 3 — bridge LiDAR:
    source /opt/ros/jazzy/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
      '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'

Terminal 4 — bridge odometry:
    source /opt/ros/jazzy/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
      '/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'

Terminal 5 — open RViz2:
    source /opt/ros/jazzy/setup.bash
    rviz2

In RViz2, add a LaserScan display for /lidar. The scan frame
observed in ROS 2 was amr/base_link/front_lidar.

## Git setup
- A dedicated Git repository was initialized in this workspace.
- .gitignore excludes build/, install/, log/, Python cache files,
  editor swap files, and OS metadata.
- The parent C:\Users\reena Git repository is separate and
  should not be used to commit this project.

## Next steps
1. Verify and commit the current working source files.
2. Add a simple LiDAR-based obstacle detector.
3. Test a safe stop response when an obstacle is close.
4. Improve TF and odometry integration for navigation.
5. Progress toward Nav2 and multi-AMR fleet coordination.

## Safety notes
- Stop continuous /cmd_vel publishers before sending a zero
  velocity command.
- Do not overwrite warehouse.sdf or amr.urdf wholesale.
- Keep backups until changes have been tested.
- Build after source/configuration changes:
    colcon build --packages-select amr_description
