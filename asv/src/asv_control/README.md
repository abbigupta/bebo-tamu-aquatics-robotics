# ASV twin-thruster control

The controller accepts independent port and starboard thrust requests in
newtons. It clamps unsafe values and sets a thruster to zero if its command is
not refreshed within 0.75 seconds.

## Gazebo acceptance test

Build the workspace once:

```bash
cd ~/ros_projects/asv
sourceros
colcon build --symlink-install
```

Start Gazebo in terminal 1:

```bash
cd ~/ros_projects/asv
./tools/run_sim.bash
```

After the ASV appears, run the automatic maneuver in terminal 2:

```bash
cd ~/ros_projects/asv
./tools/test_thrusters.bash
```

The sequence commands forward motion, a stop, an in-place turn, reverse
motion, and a final stop.

## Keyboard control

With Gazebo running, start keyboard control in another terminal:

```bash
cd ~/ros_projects/asv
sourceros
ros2 run asv_control keyboard_teleop
```

Use `W`/up for forward, `S`/down for reverse, `A`/left and `D`/right for
pivot turns, and Space or `X` to stop. Press `Q` to stop and exit. The selected
command remains active until another key is pressed, so always stop before
leaving the terminal.

## ROS interface

Command topics (`std_msgs/msg/Float64`, newtons):

- `/asv/thrusters/port/command`
- `/asv/thrusters/starboard/command`

Applied-command feedback topics (`std_msgs/msg/Float64`, newtons):

- `/asv/thrusters/port/applied`
- `/asv/thrusters/starboard/applied`

Emergency stop service:

```bash
ros2 service call /asv/thrusters/stop std_srvs/srv/Trigger '{}'
```

For a persistent manual command, publish both sides from separate terminals.
For example, 8 N on both sides drives straight:

```bash
ros2 topic pub -r 10 /asv/thrusters/port/command \
  std_msgs/msg/Float64 '{data: 8.0}'
```

```bash
ros2 topic pub -r 10 /asv/thrusters/starboard/command \
  std_msgs/msg/Float64 '{data: 8.0}'
```

Positive thrust is forward. Opposite signs produce a pivot turn. Commands
must be published continuously because the watchdog intentionally rejects
stale commands.

## Ocean current

The simulation publishes a bounded Gauss-Markov current, following the model
used by DAVE, to Gazebo's native hydrodynamics system. The vector is expressed
in the world ENU frame and the default is 0.25 m/s at 30 degrees. Change it at
launch time, for example:

```bash
./tools/run_sim.bash \
  current_speed_mps:=0.45 \
  current_direction_deg:=120.0 \
  current_speed_noise_stddev:=0.025
```

Set `current_speed_noise_stddev:=0.0` for a constant current or
`current_speed_mps:=0.0 current_speed_noise_stddev:=0.0` for still water.
The live ROS topic is `/model/asv/ocean_current` (`geometry_msgs/msg/Vector3`).

Keyboard commands now pass through a 50 Hz mux, velocity controller, and
thruster safety loop. The previous 10--20 Hz pipeline could add about
75--150 ms before Gazebo received thrust; the new path is typically under one
50 Hz control cycle plus transport time. If response is still slow, check
Gazebo's real-time factor: a value below 1.0 indicates compute/rendering load,
not buoyancy latency.
