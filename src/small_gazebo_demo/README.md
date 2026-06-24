# small_gazebo_demo

`small_gazebo_demo` is a stable fallback pipeline for demonstrating:

Gazebo simulation -> 2D LiDAR + odometry -> SLAM map -> RViz -> planner path -> Gazebo execution.

It intentionally uses a simple differential-drive robot and a small axis-aligned room. The robot publishes Gazebo world odometry and the mapping driver stops before turning, which avoids the corner-turn wall duplication artifacts seen with slipping encoder odometry in the larger Ackermann setup. Every stage of the perception-planning-control loop is still present.

## Quick command reference

```bash
# 0. Always start clean (kills any orphaned planner/gazebo/rviz from a prior run)
pkill -9 -f 'ego_planner|bspline_planner|kino_astar|small_path|small_auto'
pkill -9 -f 'gzserver|gzclient|rviz2'; pkill -9 -f 'gazebo'

# 1. Build + source
cd ~/planner_ws
colcon build --packages-select small_gazebo_demo planner_gazebo_demo --symlink-install
source install/setup.bash

# 2. Map: Gazebo -> SLAM -> RViz, then save
ros2 launch small_gazebo_demo slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/planner_ws/src/small_gazebo_demo/maps/slam_small_map

# 3. Plan + RViz playback (no Gazebo). Default map is the SLAM map.
ros2 launch small_gazebo_demo planner.launch.py planner:=ego

# 4. Full closed loop: planner path driven by the robot in Gazebo.
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego            # headless
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego gui:=true  # see the green path line in Gazebo
```

`planner` is one of `kino_astar`, `bspline`, `ego`. Run **only one** planner launch at a
time — see [Troubleshooting](#troubleshooting) if the route swings or the robot won't move.

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

The default map is the saved SLAM map, so this just works:

```bash
ros2 launch small_gazebo_demo planner.launch.py planner:=ego
```

Or pass an explicit map (e.g. the generated fallback):

```bash
ros2 launch small_gazebo_demo planner.launch.py planner:=ego \
  map_yaml:=$HOME/planner_ws/src/small_gazebo_demo/maps/small_map.yaml \
  start_x:=-2.35 start_y:=-2.35 goal_x:=2.35 goal_y:=2.35
```

`planner` can be `kino_astar`, `bspline`, or `ego`; these reuse the existing planner implementations from `planner_gazebo_demo`. This launch is RViz-only playback: `small_path_player` subscribes to `/planner/path` and publishes a moving `map -> base_link` TF plus wheel joint states, so the model moves along the route without Gazebo. The player restarts only when the path actually changes, so the constant planner republish at a few Hz no longer resets the robot to the start each cycle.

## 4. Full map -> planner -> Gazebo execution loop

```bash
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego
```

This launches Gazebo again, loads the SLAM map into the existing planner, publishes `/planner/path`, and uses `small_path_follower` to drive the simple robot along that path through `/cmd_vel`.

Coordinate frames matter here. The robot spawns at the world room corner
`(-2.35, -2.35)`, and the diff-drive plugin reports **odometry relative to the spawn
pose** — at `t=0`, `odom -> base_link` is `(0, 0)`. So the `odom` frame origin sits on
the spawn corner, and `map -> base_link` at spawn equals `map -> odom`.

The SLAM map was built starting from that same corner, so the corner is `map (0, 0)`.
That means **`map -> odom` is identity `(0, 0)`** for the SLAM map, the planner `start`
is `(0, 0)` (the corner), and `goal` is `(4.7, 4.65)` (the opposite corner). These are
the defaults. The general rule: `map_to_odom_(x,y)` should equal the planner `start`,
because the robot's spawn-relative odom is `(0,0)` and you want it to appear at `start`.
(An earlier version wrongly set `map -> odom = (2.35, 2.30)`, which shifted the robot off
the path start in RViz and made the follower stop early.)

The generated `small_map` is built directly in the world frame, so the robot's spawn
corner is `small_map (-2.35, -2.35)`; set `map_to_odom` and `start` to match:

```bash
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego \
  map_yaml:=$HOME/planner_ws/src/small_gazebo_demo/maps/small_map.yaml \
  map_to_odom_x:=-2.35 map_to_odom_y:=-2.35 \
  start_x:=-2.35 start_y:=-2.35 goal_x:=2.35 goal_y:=2.35
```

### Showing the planned route inside Gazebo

The closed loop runs `gazebo_path_marker`, which draws `/planner/path` as a green
`LINE_STRIP` marker in the Gazebo 3D view (via `gz marker`). Gazebo markers are
rendered by the Gazebo GUI, so run with the GUI on to see them:

```bash
ros2 launch small_gazebo_demo closed_loop.launch.py planner:=ego gui:=true
```

The path is published in the map frame; the marker node subtracts the
`map -> odom` offset so the line lands on the world-frame floor under the robot.
(RViz already shows the same `/planner/path` regardless of `gui`.)

> Note: `gazebo_path_marker.py` shells out to `gz marker` on purpose. Linking
> `ignition-msgs` directly into this ament package fails with a protobuf
> version clash, so the CLI (which carries Gazebo's own protobuf) is used instead.

## 5. Regenerate the fallback map

```bash
python3 src/small_gazebo_demo/scripts/small_world_to_map.py
```

## Troubleshooting

### The route swings left/right and the robot won't move

This means **more than one planner is publishing `/planner/path`**. RViz then flips
between the two paths each cycle (the "swing"), and the player/follower keeps
restarting from the start (the robot "won't move"). It is almost always an
**orphaned `ego_planner` from a previous run** — `ros2 launch` + Gazebo often leave
nodes alive after Ctrl+C in WSL.

Check and fix:

```bash
ros2 node list | grep -E 'ego|bspline|kino'      # should list the planner only ONCE
pgrep -af ego_planner                            # should show ONE process (or none)
# kill everything and re-run a single launch:
pkill -9 -f 'ego_planner|bspline_planner|kino_astar|small_path|small_auto'
pkill -9 -f 'gzserver|gzclient|rviz2'; pkill -9 -f 'gazebo'
```

### After editing the C++ nodes, behavior doesn't change

`colcon` sometimes keeps a stale binary. Force a clean rebuild of the package whose
source changed, e.g.:

```bash
rm -rf build/planner_gazebo_demo install/planner_gazebo_demo
colcon build --packages-select planner_gazebo_demo --symlink-install
source install/setup.bash
```
