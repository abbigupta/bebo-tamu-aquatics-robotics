# ASV Pi-side control

This package contains the command path that runs on the Raspberry Pi 5:

- `keyboard_teleop` publishes manual `TwistStamped` commands.
- `cmd_vel_mux` gives fresh teleoperation commands priority over autonomy and
  outputs zero when both inputs are stale.
- `mavros_velocity_adapter` forwards fresh commands to MAVROS and outputs zero
  when its input is stale.

The adapter defaults to `/mavros/setpoint_velocity/cmd_vel`. Motor mixing,
actuator limits, arming, mode management, and low-level failsafes belong on
the Pixhawk. No Gazebo thruster topics are present in this package.
