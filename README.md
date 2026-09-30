# TAMU autonomous surface vehicle

ROS 2 and Gazebo Sim stack for the TAMU Aquatics Robotics Research Lab USV.
The vessel follows predefined map-frame waypoints while carrying a separate
instrument. The instrument is intentionally not part of this control and
navigation data path.

## Quick start

The shell command `sourceros` loads ROS 2 Lyrical and this workspace from any
directory. Open a new terminal before using it for the first time, or run:

```bash
source ~/.bashrc
```

Build after changing source code:

```bash
sourceros
cd ~/ros_projects/asv
colcon build --symlink-install
```

Start the complete simulator in terminal 1:

```bash
sourceros
cd ~/ros_projects/asv
./tools/run_sim.bash
```

Run the predefined square path in terminal 2:

```bash
sourceros
ros2 launch asv_bringup square_mission.launch.py
```

The path can be resized and slowed down for bench testing:

```bash
ros2 launch asv_bringup square_mission.launch.py \
  side_length_m:=2.0 cruise_speed_mps:=0.4 acceptance_radius_m:=0.35
```

## Simulated sensors

The Gazebo model exposes the same navigation and perception categories used on
the yellow robot:

| Sensor | ROS topic | Message | Rate |
| --- | --- | --- | --- |
| GPS | `/asv/gps/fix` | `sensor_msgs/msg/NavSatFix` | 10 Hz |
| IMU | `/asv/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz |
| LiDAR | `/asv/lidar/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz |
| Forward sonar | `/asv/sonar/front` | `sensor_msgs/msg/Range` | 15 Hz |

Gazebo does not provide a native acoustic ranging sensor, so the sonar model
uses a narrow nine-ray fan and publishes its nearest return as an ultrasound
`Range`. The raw fan remains available on `/asv/sonar/scan` for debugging.
GPS data passes through a covariance adapter before localization.

The dual-filter localization chain publishes local odometry on
`/asv/odometry/local` and GPS-corrected map odometry on
`/asv/odometry/filtered`. The waypoint action server uses the filtered output,
then sends velocity commands through a priority/timeout mux and the
twin-thruster controller.

Inspect live data with, for example:

```bash
ros2 topic echo /asv/gps/fix --once
ros2 topic echo /asv/imu/data --once
ros2 topic hz /asv/lidar/scan
ros2 topic echo /asv/sonar/front --once
```

## Validation

```bash
sourceros
cd ~/ros_projects/asv
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Mass, inertia, hull, and propulsion values in
`asv_description/config/physics_real.yaml` are calibration placeholders. Use
measured, fully loaded vessel values before treating controller performance as
representative of the physical USV.
