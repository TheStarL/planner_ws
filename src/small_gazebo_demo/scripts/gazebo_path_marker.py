#!/usr/bin/env python3
"""Draw the planner path as a LINE_STRIP marker inside the Gazebo 3D view.

/planner/path is published in the (SLAM) map frame, while Gazebo renders in the
world frame. The two differ by the same offset closed_loop.launch.py puts on
map->odom, so we subtract it here: world = map - offset. Pass marker_x_offset =
-map_to_odom_x (and y) to match the launch.

This shells out to `gz marker`, which links Gazebo's own protobuf (avoiding the
ABI clash from linking ignition-msgs into an ament build). Markers are served by
gzclient, so the path is only visible with gui:=true.
"""

import shutil
import subprocess

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class GazeboPathMarker(Node):
    def __init__(self):
        super().__init__("gazebo_path_marker")
        self.z_height = self.declare_parameter("z_height", 0.06).value
        self.line_width = self.declare_parameter("line_width", 0.04).value
        self.x_offset = self.declare_parameter("marker_x_offset", -2.35).value
        self.y_offset = self.declare_parameter("marker_y_offset", -2.30).value

        self._gz = shutil.which("gz")
        if self._gz is None:
            self.get_logger().error("`gz` not found on PATH; cannot draw markers.")

        self._points = []
        self.create_subscription(Path, "/planner/path", self._path_cb, 10)
        # Re-send periodically so the path still appears if gzclient connects
        # after the path was first published (ADD_MODIFY just replaces it).
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            "gazebo_path_marker ready (needs gui:=true to be visible).")

    def _path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
        self._points = pts
        self._publish()

    def _publish(self):
        if self._gz is None or len(self._points) < 2:
            return
        parts = [
            "action: ADD_MODIFY",
            "type: LINE_STRIP",
            'ns: "planner_path"',
            "id: 1",
            "visibility: GUI",
            "scale: {x: %g, y: %g, z: %g}" % (
                self.line_width, self.line_width, self.line_width),
            "material: {ambient: {r: 0.05, g: 0.85, b: 0.1, a: 1}, "
            "diffuse: {r: 0.05, g: 0.85, b: 0.1, a: 1}}",
        ]
        for x, y in self._points:
            parts.append("point: {x: %g, y: %g, z: %g}" % (
                x + self.x_offset, y + self.y_offset, self.z_height))
        msg = ", ".join(parts)
        try:
            subprocess.run([self._gz, "marker", "-m", msg],
                           check=False, capture_output=True, timeout=5)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("gz marker failed: %s" % exc)


def main():
    rclpy.init()
    node = GazeboPathMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
