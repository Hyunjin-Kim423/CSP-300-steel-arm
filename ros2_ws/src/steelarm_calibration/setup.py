from setuptools import find_packages, setup

package_name = 'steelarm_calibration'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', ['launch/calibration.launch.py', 'launch/synthetic_replay.launch.py', 'launch/webcam_test.launch.py']),
                  ('share/' + package_name + '/config', ['config/apriltag.yaml', 'config/tag_mount.yaml', 'config/bridge.yaml', 'config/synthetic_apriltag.yaml', 'config/synthetic_apriltag_ros.yaml', 'config/apriltag_ros_candidate.yaml', 'config/webcam_test_apriltag.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='SteelArm', maintainer_email='todo@example.invalid', license='MIT',
      entry_points={'console_scripts': [
          'camera_pose_tf = steelarm_calibration.camera_pose_tf:main',
          'tag_base_tf = steelarm_calibration.tag_base_tf:main',
          'calibration_validator = steelarm_calibration.calibration_validator:main',
          'pose_logger = steelarm_calibration.pose_logger:main',
          'synthetic_replay = steelarm_calibration.synthetic_replay:main',
          'webcam_bridge = steelarm_calibration.webcam_bridge:main',
      ]})
