"""Run after sourcing Jazzy. Uses unique fake robots in a localhost test domain."""
import json
import os
import time

import numpy as np
import pytest
import yaml

os.environ.setdefault('ROS_DOMAIN_ID', '182')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_STATIC_PEERS'] = ''
rclpy = pytest.importorskip('rclpy')
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from mir_msgs.msg import RobotState
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from match_mir_camera_calibration.dataset import load_dataset
from match_mir_camera_calibration.session_node import CalibrationSession, PREFIX


def test_ros_capture_lossless_frames_clock_rejection_and_no_motion(config, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    config['target_robot'], config['observer_robot'] = 'calibration_test_a', 'calibration_test_b'
    path = tmp_path/'settings.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init(args=['--ros-args', '-p', f'config_path:={path}', '-p', 'owner_token:=test_owner'])
    session, feeder, executor = None, None, None
    try:
        session = CalibrationSession()
        feeder = rclpy.create_node('calibration_test_feeder')
        executor = SingleThreadedExecutor()
        executor.add_node(session)
        executor.add_node(feeder)
        poses = {r: feeder.create_publisher(PoseStamped, f'/qualisys/{r}/pose', 100)
                 for r in (config['target_robot'], config['observer_robot'])}
        states = {r: feeder.create_publisher(RobotState, f'/{r}/robot_state', 10) for r in poses}
        heart = feeder.create_publisher(String, PREFIX+'/heartbeat', 1)
        images, infos = {}, {}
        commands = []
        feeder.create_subscription(TwistStamped, '/calibration_test_a/cmd_vel_stamped',
                                   lambda msg: commands.append((msg.twist.linear.x, msg.twist.angular.z)), 10)
        for side in ('left', 'right'):
            base = f'/calibration_test_b/camera_floor_{side}/driver/color'
            images[side] = feeder.create_publisher(Image, base+'/image_raw', qos_profile_sensor_data)
            infos[side] = feeder.create_publisher(CameraInfo, base+'/camera_info', qos_profile_sensor_data)
        bridge = CvBridge()
        pixel = np.full((480, 640, 3), 255, np.uint8)
        import cv2
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        marker = cv2.aruco.drawMarker(dictionary, 7, 180) if hasattr(cv2.aruco, 'drawMarker') else cv2.aruco.generateImageMarker(dictionary, 7, 180)
        pixel[140:320, 230:410] = marker[..., None]
        def publish_poses():
            for robot, publisher in poses.items():
                msg = PoseStamped()
                msg.header.frame_id = 'mocap'
                msg.header.stamp = feeder.get_clock().now().to_msg()
                msg.pose.position.x = 2.0 if robot == config['target_robot'] else 0.0
                msg.pose.orientation.w = 1.0
                publisher.publish(msg)
        def publish_state():
            for pub in states.values():
                pub.publish(RobotState(robot_state=3))
            heart.publish(String(data='test_owner'))
        def publish_images():
            for side in images:
                stamp = feeder.get_clock().now().to_msg()
                frame = f'calibration_test_b/{side}_optical'
                info = CameraInfo()
                info.header.stamp, info.header.frame_id = stamp, frame
                info.height, info.width = 480, 640
                info.distortion_model = 'plumb_bob'
                info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
                info.d = [0.]*5
                infos[side].publish(info)
                msg = bridge.cv2_to_imgmsg(pixel, encoding='bgr8')
                msg.header.stamp, msg.header.frame_id = stamp, frame
                images[side].publish(msg)
        feeder.create_timer(.01, publish_poses)
        feeder.create_timer(.1, publish_state)
        feeder.create_timer(.5, publish_images)
        def spin_until(condition, timeout=12.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
                if condition():
                    return
            raise AssertionError(f'Timeout: {session.controller.state}: {session.controller.reason}')
        def call(action):
            client = feeder.create_client(Trigger, PREFIX+'/'+action)
            assert client.wait_for_service(timeout_sec=1.)
            future = client.call_async(Trigger.Request())
            spin_until(future.done, 3.)
            return future.result()
        spin_until(lambda: len(session.images) == 2 and len(session.controller.poses) == 2)
        assert call('prepare').success
        reply = call('start')
        assert not reply.success and 'motion_enabled' in reply.message
        assert call('capture').success
        spin_until(lambda: session.controller.state == 'STOPPED' and session.dataset is not None and session.dataset.count == 1)
        _, _, _, measurements = load_dataset(session.dataset.path)
        assert len(measurements) == 1
        assert len(measurements[0]['images']) == 10
        assert all(image['detections'][0]['id'] == 7 for image in measurements[0]['images'])
        image = cv2.imread(str(session.dataset.path/measurements[0]['images'][0]['path']))
        assert np.array_equal(image, pixel)
        assert len((session.dataset.path/'poses.jsonl').read_text().splitlines()) > 100
        assert call('capture').success
        spin_until(lambda: session.dataset.pending is not None)
        bad = bridge.cv2_to_imgmsg(pixel, encoding='bgr8')
        bad.header.frame_id = 'calibration_test_b/left_optical'
        bad.header.stamp.sec = 1
        images['left'].publish(bad)
        spin_until(lambda: session.controller.state == 'FAULT')
        spin_until(lambda: session.dataset.pending is None)
        rejected = json.loads((session.dataset.path/'measurements/000001/measurement.json').read_text())
        assert not rejected['accepted'] and 'timestamp' in rejected['reason']
        # The watchdog must run even when the ROS executor itself is blocked.
        spin_until(lambda: len(session.camera_seen) == 2)
        assert call('capture').success
        time.sleep(.3)
        assert session.watchdog_tripped.is_set()
        spin_until(lambda: session.controller.state == 'FAULT')
        assert commands and all(v == 0. and w == 0. for v, w in commands)
    finally:
        if session:
            session.destroy_node()
        if feeder:
            feeder.destroy_node()
        if executor:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
