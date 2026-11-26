#!/usr/bin/env python3
"""TEST: selected object (from the Quest app) -> /object_target_point for pointer_node.

/quest/selected_object (PointStamped, quest_world)  --LOCK (base_link <- quest_world)-->  base_link
Then, for an object on the desk side (x < 0, behind the robot), the point is mirrored to (-x, -y, z):
pointer_node turns its pan toward atan2(y, x), and the arm's hand points to -x at pan 0, so the mirrored
point makes the HAND face the object. Run pointer_node with target_side:=1 (target in front of the hand).
Height and horizontal distance do not change, so the vertical aim is the same.

It republishes the last point at 10 Hz while new points keep coming (stops 1 s after the last one).
Started by the dashboard (tools/steelarm/dashboard.py) in object modes.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class ObjectTarget(Node):
    def __init__(self):
        super().__init__('object_target')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointStamped, '/quest/selected_object', self.on_object, 10)
        self.pub = self.create_publisher(PointStamped, '/object_target_point', 10)
        self.last = None            # (x, y, z) in base_link, before mirroring
        self.last_time = None       # local arrival time (s)
        self.state = 'waiting'
        self.create_timer(0.1, self.tick)
        self.get_logger().info('waiting for /quest/selected_object (select an object in the app)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_object(self, msg):
        try:
            # The lock is a static TF, so the latest one is the right one.
            t = self.buffer.lookup_transform('base_link', msg.header.frame_id or 'quest_world', Time())
        except TransformException as e:
            self.set_state('no_lock', f'no base_link <- quest_world TF (is run_head_target.sh LOCKED?): {e}')
            return
        tr, ro = t.transform.translation, t.transform.rotation
        r = rotate((ro.x, ro.y, ro.z, ro.w), (msg.point.x, msg.point.y, msg.point.z))
        p = (r[0] + tr.x, r[1] + tr.y, r[2] + tr.z)
        new_object = self.last is None or math.dist(p, self.last) > 0.15
        self.last, self.last_time = p, self.now()
        dist = math.hypot(p[0], p[1])
        bearing = math.degrees(math.atan2(p[1], p[0]))
        side = 'desk side (behind the robot)' if p[0] < 0 else 'USER side (this test mode expects the desk side)'
        if new_object or self.state != 'sending':
            self.set_state('sending', f'object at x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f} m: '
                                      f'{dist:.2f} m away, {bearing:+.0f} deg, {side}')

    def set_state(self, state, text):
        # Note: rclpy does not allow one logging line to switch between info and warn, so two separate calls.
        if state == 'sending':
            self.get_logger().info(text)
        elif state != self.state:
            self.get_logger().warn(text)
        self.state = state

    def tick(self):
        if self.last is None:
            return
        if self.now() - self.last_time > 1.0:
            if self.state == 'sending':
                self.set_state('stopped', 'no new object point for 1 s (deselected or lost): stopped')
            return
        x, y, z = self.last
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.point.x, out.point.y, out.point.z = (-x, -y, z) if x < 0 else (x, y, z)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObjectTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
