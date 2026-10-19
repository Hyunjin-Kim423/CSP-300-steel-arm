"""Publish a USB webcam as a test-only ROS 2 camera source.

This node intentionally uses a separate namespace and frame from Quest.  The
default field of view is an explicitly provisional estimate; use a calibrated
camera-info YAML before interpreting a detected tag pose in metres.
"""
import math

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class WebcamBridge(Node):
    def __init__(self):
        super().__init__('webcam_bridge')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'webcam_optical_test')
        self.declare_parameter('image_topic', '/test/webcam/image_raw')
        self.declare_parameter('camera_info_topic', '/test/webcam/camera_info')
        self.declare_parameter('provisional_horizontal_fov_degrees', 70.0)
        self.declare_parameter('fourcc', 'MJPG')

        device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = self.get_parameter('frame_id').value
        fourcc = self.get_parameter('fourcc').value.upper()
        if len(fourcc) != 4:
            raise ValueError("fourcc must contain exactly four characters, for example 'MJPG'")
        fov = float(self.get_parameter('provisional_horizontal_fov_degrees').value)
        if not 1.0 < fov < 179.0:
            raise ValueError('provisional_horizontal_fov_degrees must be between 1 and 179')

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # The Office G1080 is a UVC 1.00 device.  Its uncompressed default
        # stream can stall under USB/IP; request MJPEG explicitly and retain
        # only the freshest frame for this test path.
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, float(self.get_parameter('fps').value))
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise RuntimeError(f'Could not open webcam {device}. Check /dev/video* and USB/IP attachment.')

        self.image_pub = self.create_publisher(Image, self.get_parameter('image_topic').value, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.get_parameter('camera_info_topic').value, 10)
        self.fx = self.width / (2.0 * math.tan(math.radians(fov) / 2.0))
        self.fy = self.fx
        period = 1.0 / float(self.get_parameter('fps').value)
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().warning(
            f'TEST ONLY: using provisional {fov:.1f} degree horizontal FOV for CameraInfo. '
            'Calibrate this webcam before using tag pose measurements.'
        )
        self.get_logger().info(
            'Webcam opened: '
            f"{int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
            f"@ {self.capture.get(cv2.CAP_PROP_FPS):.1f} fps, requested FOURCC={fourcc}."
        )

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('Webcam frame read failed.')
            return
        height, width = frame.shape[:2]
        if (width, height) != (self.width, self.height):
            self.width, self.height = width, height
            self.fx = self.width / (2.0 * math.tan(math.radians(float(self.get_parameter('provisional_horizontal_fov_degrees').value)) / 2.0))
            self.fy = self.fx
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = 'bgr8'
        image.is_bigendian = False
        image.step = self.width * 3
        image.data = frame.tobytes()
        self.image_pub.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.width / 2.0, 0.0, self.fy, self.height / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.width / 2.0, 0.0, 0.0, self.fy, self.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def destroy_node(self):
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = WebcamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
