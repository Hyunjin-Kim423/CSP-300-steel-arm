"""Detect the robot tag until the result is stable. Then freeze base_link -> quest_world as a static TF.

The tag chain must be running (camera_pose_tf, tag_base_tf, apriltag_node):
quest_world -> quest_camera_optical -> robot_tag_0 -> robot_base.

How it works:
- Each NEW detection (new stamp on quest_camera_optical -> robot_tag_0) gives one sample of robot_base in quest_world.
  We look up the sample at the stamp of the detection. So a moving head does not make the sample blurry.
- When the last `window` samples agree (see StableWindow), we invert the mean
  and publish it one time as a static TF `base_link -> quest_world`. robot_base is used as base_link, same as tag mode.
- After that, we ignore detections. So apriltag_node can be stopped.

Keep this node running. A static TF goes to late nodes only while this node is alive.
To align again, restart this node.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .align_once_logic import StableWindow, invert_pose, level_pose, up_cos


class TagAlignOnce(Node):
    def __init__(self):
        super().__init__('tag_align_once')
        self.declare_parameter('world_frame', 'quest_world')
        self.declare_parameter('camera_frame', 'quest_camera_optical')
        self.declare_parameter('tag_frame', 'robot_tag_0')
        # The robot base frame. I measured it in tag_mount.yaml.
        self.declare_parameter('base_source_frame', 'robot_base')
        # We publish parent_frame -> world_frame.
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('window', 10)
        self.declare_parameter('pos_tol_m', 0.02)
        self.declare_parameter('ang_tol_deg', 3.0)
        self.declare_parameter('check_hz', 20.0)
        # Gravity check. quest_world is aligned with gravity, and the robot is on a flat desk.
        # - If robot_base +z of a sample is more than acos(min_up_cos) away from up, it is a wrong or flipped detection. We drop it.
        # - We also level the lock (we keep only the yaw).
        self.declare_parameter('min_up_cos', 0.9)
        self.declare_parameter('level_lock', True)

        def get(name):
            return self.get_parameter(name).value

        self.world, self.cam, self.tag = get('world_frame'), get('camera_frame'), get('tag_frame')
        self.src, self.parent = get('base_source_frame'), get('parent_frame')
        self.win = StableWindow(int(get('window')), float(get('pos_tol_m')), float(get('ang_tol_deg')))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.min_up_cos, self.level = float(get('min_up_cos')), bool(get('level_lock'))
        self.last_stamp_ns = None
        self.locked = False
        self.rejected = 0
        self.last_log_ns = 0
        self.timer = self.create_timer(1.0 / float(get('check_hz')), self.step)
        self.get_logger().info(
            f'Waiting for stable {self.tag} detections: {self.win.size} samples within '
            f'{self.win.pos_tol_m * 100:.1f} cm / {self.win.ang_tol_deg:.1f} deg. Hold the headset still, tag in view.')

    def step(self):
        if self.locked:
            return
        # Get the latest detection. We use it only for the stamp.
        try:
            det = self.buffer.lookup_transform(self.cam, self.tag, Time())
        except Exception:
            return
        stamp = Time.from_msg(det.header.stamp)
        # Skip the detection if we already used it.
        if stamp.nanoseconds == self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp.nanoseconds
        try:
            tf = self.buffer.lookup_transform(self.world, self.src, stamp, timeout=Duration(seconds=0.05))
        except Exception as error:
            # The camera pose of that stamp did not come yet.
            self.get_logger().debug(f'skip sample: {error}')
            return
        tr, ro = tf.transform.translation, tf.transform.rotation
        # If the sample is tilted against gravity, it is not a real alignment.
        if up_cos((ro.x, ro.y, ro.z, ro.w)) < self.min_up_cos:
            self.rejected += 1
            return
        self.win.add((tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w))
        result = self.win.result()
        if result is None:
            self._progress()
            return
        self._lock(*result)

    def _progress(self):
        """Log the progress. At most one time in 2 seconds."""
        now = self.get_clock().now().nanoseconds
        if now - self.last_log_ns < 2_000_000_000:
            return
        self.last_log_ns = now
        n = len(self.win.samples)
        if n < self.win.size:
            self.get_logger().info(f'collecting samples {n}/{self.win.size} ({self.rejected} tilted samples rejected)')
        else:
            pos, ang = self.win.spread()
            self.get_logger().info(
                f'samples disagree: {len(self.win.inliers())}/{self.win.size} agree, need {self.win.min_inliers} '
                f'(best group spread {pos * 100:.1f} cm, {ang:.1f} deg); hold still')

    def _lock(self, t_world_base, q_world_base):
        if self.level:
            lv = level_pose(t_world_base, q_world_base)
            if lv is None:
                self.get_logger().warn('robot_base +x is vertical, cannot level the lock; keep collecting')
                self.win.samples.clear()
                return
            t_world_base, q_world_base = lv
        # Pose of the world in the base frame.
        t, q = invert_pose(t_world_base, q_world_base)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = self.parent, self.world
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = t
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        self.broadcaster.sendTransform(msg)
        self.locked = True
        self.timer.cancel()
        pos, ang = self.win.spread()
        self.get_logger().info(
            f'LOCKED static TF {self.parent} -> {self.world}: xyz=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m, '
            f'q=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}); {len(self.win.inliers())}/{self.win.size} samples used, '
            f'spread {pos * 100:.1f} cm / {ang:.1f} deg. '
            f'Detection no longer needed.')


def main():
    rclpy.init()
    node = TagAlignOnce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
