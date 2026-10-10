# BEBO TAMU Aquatics Robotics — Raspberry Pi 5

This repository is the deployable ROS 2 Lyrical workspace for the Raspberry
Pi 5 aboard the BEBO autonomous surface vehicle. It connects to the Pixhawk
through MAVROS and contains only code intended to run on physical hardware.

Gazebo worlds, simulation launch files, simulated sensor adapters, simulated
ocean current, ground-truth localization, meshes, and Gazebo thruster plugins
are intentionally excluded.

## Runtime layout

```text
Pixhawk -- MAVLink --> MAVROS -- odometry --> waypoint follower
                                  ^                 |
                                  |                 v
                           velocity setpoint <-- command mux
                                                   ^
                                          autonomy or teleop
```

The Pixhawk remains responsible for its flight-controller safety behavior,
state estimation, motor output, arming, and vehicle mode. The Pi does not arm
the vehicle or change modes automatically.

## Pi 5 setup

Install ROS 2 Lyrical and the ROS dependencies, then clone this repository as
a workspace:

```bash
git clone https://github.com/abbigupta/bebo-tamu-aquatics-robotics.git
cd bebo-tamu-aquatics-robotics
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./tools/build.bash
```

The optional BME280 driver also needs the CircuitPython libraries:

```bash
python3 -m pip install -r requirements-pi5.txt
```

Add the runtime user to `dialout` and log out and back in before opening the
Pixhawk or TFmini serial ports:

```bash
sudo usermod -aG dialout "$USER"
```

Enable the Pi UART and verify the Pixhawk TELEM port wiring and voltage levels
before powering the system. The default MAVROS connection is
`/dev/serial0:115200`; it can be overridden at launch.

## Start hardware

With propellers physically made safe for a bench test:

```bash
./tools/run_hardware.bash
```

Example with a different Pixhawk port and all supported Pi sensors enabled:

```bash
./tools/run_hardware.bash \
  fcu_url:=/dev/ttyAMA0:921600 \
  start_tfmini:=true tfmini_port:=/dev/ttyAMA3 \
  start_bme280:=true \
  start_ds18b20:=true ds18b20_device_id:=28-000000230f20
```

Confirm the link and live position before sending commands:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/odom --once
```

If the Pixhawk odometry uses another topic, pass
`odometry_topic:=/your/topic` to both hardware and mission launches.

## Manual control

After the hardware launch is running, use a second terminal:

```bash
./tools/run_keyboard_teleop.bash
```

`W`/up and `S`/down command surge, `A`/left and `D`/right command yaw, Space
or `X` stops, and `Q` stops and exits. Teleoperation has priority over
autonomy while its commands are fresh. The command mux and MAVROS adapter
publish zero commands when input goes stale.

## Square mission

Only run missions after the vehicle is armed in the correct ArduPilot mode,
the MAVROS odometry is stable, the operating area is clear, and an independent
emergency stop is available.

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch asv_bringup square_mission.launch.py \
  side_length_m:=5.0 cruise_speed_mps:=0.7 acceptance_radius_m:=0.5
```

The mission uses the current Pixhawk local position as the first corner and
returns to that position.

## Packages

- `asv_bringup`: Pi/Pixhawk and mission launch files.
- `asv_control`: teleoperation, command priority/timeout mux, and MAVROS
  velocity adapter.
- `asv_autonomy`: waypoint action server and square mission client.
- `asv_interfaces`: custom waypoint action and shared ASV messages.
- `asv_sensors`: physical TFmini Plus, DS18B20, and BME280 drivers.

## Validation

```bash
source /opt/ros/lyrical/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
