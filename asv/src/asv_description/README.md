# ASV description

This package provides a stable first path from the Fusion 360 assembly to
Gazebo Sim. It deliberately keeps the detailed visual mesh separate from the
simple collision geometry used for physics and buoyancy.

## Fast path: one combined OBJ with materials

1. In Fusion 360, orient the assembly with **X forward, Y left, Z up**.
2. Export the complete, rigid assembly as OBJ in millimetres, including its MTL.
3. Name the files `asv_visual.obj` and `asv_visual.mtl`, then copy them to:

   `meshes/visual/`

4. Build and launch from the workspace root:

   ```bash
   colcon build
   source install/setup.bash
   ros2 launch asv_bringup sim.launch.py use_mesh:=true
   ```

The OBJ's `mtllib` line must read `mtllib asv_visual.mtl`. The default mesh
scale is `0.001`, converting millimetres to metres. Override
it when necessary:

```bash
ros2 launch asv_bringup sim.launch.py use_mesh:=true mesh_scale:=1.0
```

ROS expects X forward, Y left, and Z up. The supplied Fusion STL is Y-long, so
the launch file now applies a 90-degree yaw by default. Override it if needed:

```bash
ros2 launch asv_bringup sim.launch.py use_mesh:=true mesh_yaw:=0.0
```

`mesh_x`, `mesh_y`, `mesh_z`, `mesh_roll`, `mesh_pitch`, and `mesh_yaw` are
available for visual alignment. All translations are metres and rotations are
radians.

When launching from the Snap build of VS Code, use the workspace wrapper to
remove Snap GUI library variables before Gazebo starts:

```bash
./tools/run_sim.bash
```

The mesh is visual only. Two primitive pontoons remain the collision and
buoyancy geometry so an overly detailed STL cannot destabilize physics.

The simulation configuration places the center of gravity and both thrust
axes at `z=-0.16 m`. This removes the artificial pitch-up moment that occurred
when thrust was applied below the configured center of mass. Treat this as a
simulation baseline: update `physics_real.yaml` only from measured mass,
center-of-gravity, and inertia data.

The world uses a large low-roughness PBR ocean plane, atmospheric fog, and a
visible sea floor. A collision-enabled shore begins roughly 35 m north of the
spawn point, with a submerged shelf, sloped beach, elevated grassy land, and
rocky outcrops. Physics remain independent of the visual water plane: Gazebo's
graded-buoyancy system defines the free surface at `z=0`, and the native
hydrodynamics system computes drag against the configured ocean-current
vector. This follows VRX's separation of water visuals from vehicle forces.

To use the older STL without colors:

```bash
ros2 launch asv_bringup sim.launch.py \
  use_mesh:=true mesh_file:=asv_visual.stl
```

## Full fusion2URDF path

fusion2URDF runs inside Fusion 360 and exports a complete ROS package; it does
not infer an assembly from an arbitrary folder of STL files. Export the package
and copy the whole generated `<robot>_description` directory into this
workspace's `src/` directory. Keep it beside `asv_description`, not inside it.

After `colcon build`, first run the generated package's RViz
display launch file and inspect its frames, scale, inertias, and joints. Its
URDF can then replace `urdf/asv.urdf.xacro` while this package's Gazebo world
and the `asv_bringup` launch package are retained.

## Important placeholders

The current mass, inertia, pontoon dimensions, model origin, and hydrodynamic
damping are starter values. Replace them with fully loaded vessel measurements
before evaluating controller performance. Keep measured values in
`config/physics_real.yaml`; the simulator defaults to `physics_sim.yaml` so
simulation tuning does not silently overwrite the physical configuration.
