# small_gazebo_demo

`small_gazebo_demo` is a stable fallback pipeline for demonstrating:

Gazebo simulation -> 2D LiDAR + odometry -> SLAM map -> RViz -> planner path -> Gazebo execution.

It intentionally uses a simple differential-drive robot and a small axis-aligned room. The robot publishes Gazebo world odometry and the mapping driver stops before turning, which avoids the corner-turn wall duplication artifacts seen with slipping encoder odometry in the larger Ackermann setup. Every stage of the perception-planning-control loop is still present.

## 1. Build

```bash
cd ~/planner_ws
colcon build --packages-select small_gazebo_demo planner_gazebo_demo --symlink-install
source install/setup.bash
```

## 2. Gazebo -> SLAM -> RViz

```bash
ros2 launch small_gazebo_demo slam.launch.py
```

This starts:

- Gazebo Classic with `worlds/small_room.world`
- a small differential-drive robot with `/scan`, stable `/odom`, and TF
- `slam_toolbox`
- `small_auto_mapper`, a slow deterministic stop-turn-go waypoint sweep
- RViz with `/map`, `/scan`, TF, and RobotModel

Save the SLAM map when RViz shows a clean room outline:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/planner_ws/src/small_gazebo_demo/maps/slam_small_map
```

## 3. Plan on the saved map

Use the saved SLAM map:

```bash
ros2 launch small_gazebo_demo planner.launch.py planner:=ego \
  map_yaml:=$HOME/planner_ws/src/small_gazebo_demo/maps/slam_small_map.yaml
```

Or use the generated fallback map:

```bash
ros2 launch small_gazebo_demo planner.launch.py planner:=ego
```

`planner` can be `kino_astar`, `bspline`, or `ego`; these reuse the existing planner implementations from `planner_gazebo_demo`. This launch is RViz-only playback: `small_path_player` subscribes to `/planner/path` and publishes a moving `map -> base_link` TF plus wheel joint states, so the model moves along the route without Gazebo.

## 4. Full map -> planner -> Gazebo execution loop

```bash
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego
```

This launches Gazebo again, loads the small map into the existing planner, publishes `/planner/path`, and uses `small_path_follower` to drive the simple robot along that path through `/cmd_vel`.

The closed-loop launch uses an identity `map -> odom` transform because the robot odometry is already world-aligned. Do not add the spawn offset to `map -> odom`; that double-shifts the robot and makes the model appear wrong.

## 5. Regenerate the fallback map

```bash
python3 src/small_gazebo_demo/scripts/small_world_to_map.py
```
