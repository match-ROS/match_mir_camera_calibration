"""Independent ROS recorder/controller. Never moves on launch or preparation."""
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import math
import queue
import signal
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from mir_msgs.msg import RobotState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .config import load
from .controller import ACTIVE, SessionController
from .dataset import Dataset
from .geometry import transform, values
from .vision import Detector, camera_model


PREFIX = '/mir_camera_calibration'


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def info_dict(info):
    return {'stamp': stamp_seconds(info.header.stamp), 'frame_id': info.header.frame_id,
            'height': info.height, 'width': info.width, 'distortion_model': info.distortion_model,
            'k': list(info.k), 'd': list(info.d), 'r': list(info.r), 'p': list(info.p),
            'binning_x': info.binning_x, 'binning_y': info.binning_y,
            'roi': {'x_offset': info.roi.x_offset, 'y_offset': info.roi.y_offset,
                    'height': info.roi.height, 'width': info.roi.width,
                    'do_rectify': info.roi.do_rectify}}


class CalibrationSession(Node):
    def __init__(self):
        super().__init__('mir_camera_calibration_session')
        config_path = str(self.declare_parameter('config_path', '').value)
        self.owner = str(self.declare_parameter('owner_token', '').value)
        if not config_path or not self.owner:
            raise ValueError('config_path and a GUI owner_token are required')
        self.c = load(config_path)
        self.controller = SessionController(self.c)
        self.bridge = CvBridge()
        self.detectors = {side: Detector(self.c) for side in ('left', 'right')}
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.results = queue.Queue(maxsize=2)
        self.busy = {side: False for side in ('left', 'right')}
        self.infos, self.images, self.raw_headers, self.robot_states = {}, {}, {}, {}
        self.camera_seen = {}
        self.source_stamps = {}
        self.waypoints = None
        self.dataset = None
        self.capture_started = None
        self.capture_mono = None
        self.counts = {'left': 0, 'right': 0}
        self.last_state = None
        self.watchdog_stop = threading.Event()
        self.watchdog_armed = threading.Event()
        self.watchdog_tripped = threading.Event()
        self.last_tick_mono = time.monotonic()
        self.command_publisher = (self.create_publisher(
            TwistStamped, f"/{self.c['target_robot']}/cmd_vel_stamped", 1)
            if self.c['acquisition_mode'] == 'automatic' else None)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, PREFIX+'/status', qos)
        self.preview_publishers = {side: self.create_publisher(Image, PREFIX+f'/preview/{side}', qos_profile_sensor_data)
                                   for side in ('left', 'right')}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for robot in (self.c['target_robot'], self.c['observer_robot']):
            self.create_subscription(PoseStamped, f'/qualisys/{robot}/pose', partial(self._pose, robot), 100)
            if self.c['acquisition_mode'] == 'automatic':
                self.create_subscription(RobotState, f'/{robot}/robot_state', partial(self._robot_state, robot), 10)
        for side in ('left', 'right'):
            base = f"/{self.c['observer_robot']}/camera_floor_{side}/driver/color"
            self.create_subscription(CameraInfo, base+'/camera_info', partial(self._info, side), qos_profile_sensor_data)
            self.create_subscription(Image, base+'/image_raw', partial(self._image, side), qos_profile_sensor_data)
        self.create_subscription(String, PREFIX+'/heartbeat', self._heartbeat, 1)
        self.create_subscription(String, PREFIX+'/waypoints', self._waypoints, 1)
        for name in ('prepare', 'start', 'pause', 'stop', 'capture', 'verify'):
            self.create_service(Trigger, PREFIX+'/'+name, partial(self._service, name))
        self.create_timer(0.05, self._tick)
        self.create_timer(0.2, self._status)
        self.watchdog = threading.Thread(target=self._guard, name='calibration-command-watchdog', daemon=True)
        self.watchdog.start()
        self.get_logger().info('Manual acquisition loaded: no cmd_vel publisher; use stationary capture.'
                               if self.command_publisher is None else
                               'Session loaded. Motion is idle; prepare/preview before Start.')

    def _heartbeat(self, msg):
        if msg.data == self.owner:
            self.controller.heartbeat_at = time.monotonic()

    def _pose(self, robot, msg):
        now = time.monotonic()
        stamp = stamp_seconds(msg.header.stamp)
        wall = self.get_clock().now().nanoseconds*1e-9
        if msg.header.frame_id != 'mocap' or not 0 < stamp or not -0.05 <= wall-stamp <= self.c['mocap_timeout_sec']:
            self._reject(f'Invalid frame or stale host timestamp in raw mocap: {robot}')
            return
        if stamp <= self.raw_headers.get(robot, {}).get('stamp', -math.inf):
            self._reject(f'Non-increasing raw mocap timestamp: {robot}')
            return
        try:
            v = pose_values(msg.pose)
            t = transform(v)
        except ValueError:
            self._reject(f'Invalid mocap transform: {robot}')
            return
        self.controller.observe(robot, t, now)
        self.raw_headers[robot] = {'stamp': stamp, 'frame_id': msg.header.frame_id, 'received_at': wall}
        if self.dataset:
            self.dataset.pose(dict(robot=robot, pose=v, **self.raw_headers[robot]))

    def _robot_state(self, robot, msg):
        self.robot_states[robot] = (time.monotonic(), int(msg.robot_state))

    def _info(self, side, msg):
        try:
            data = info_dict(msg)
            camera_model(data)
            self.infos[side] = data
        except ValueError as exc:
            self.infos.pop(side, None)
            self._reject(f'{side}: {exc}')

    def _image(self, side, msg):
        wall = self.get_clock().now().nanoseconds*1e-9
        source_stamp = stamp_seconds(msg.header.stamp)
        previous = self.source_stamps.get(side, -math.inf)
        if source_stamp <= previous or not -0.05 <= wall-source_stamp <= self.c['image_max_age_sec']:
            self.camera_seen.pop(side, None)
            self._reject(f'{side}: stale/backwards camera timestamp or clock offset; check MiR/PC clock synchronization')
            return
        self.source_stamps[side] = source_stamp
        self.camera_seen[side] = {'source_stamp': source_stamp, 'received_at': wall,
                                  'received_mono': time.monotonic(), 'age_sec': wall-source_stamp}
        info = self.infos.get(side)
        if not info or info['frame_id'] != msg.header.frame_id:
            self._reject(f'{side}: missing CameraInfo or optical frame mismatch')
            return
        if self.busy[side]:
            return
        self.busy[side] = True
        poses = {robot: values(item[0]) for robot, item in self.controller.poses.items()}
        headers = {robot: dict(header) for robot, header in self.raw_headers.items()}
        capture_id = self.dataset.pending['id'] if self.dataset and self.dataset.pending else None
        received_mono = time.monotonic()
        future = self.pool.submit(self._decode, side, msg, dict(info), poses, headers,
                                  wall, received_mono, capture_id)
        def done(result):
            try:
                self.results.put_nowait((side, result.result(), None))
            except Exception as exc:
                try:
                    self.results.put_nowait((side, None, str(exc)))
                except queue.Full:
                    self.busy[side] = False
        future.add_done_callback(done)

    def _decode(self, side, msg, info, poses, headers, received_at, received_mono, capture_id):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.detectors[side].detect(image, info)
        overlay = self.detectors[side].overlay(image, detections, info, self.c['preview_rotate_ccw'])
        record = {'camera': side, 'frame_id': msg.header.frame_id,
                  'stamp': stamp_seconds(msg.header.stamp), 'received_at': received_at,
                  'source_encoding': msg.encoding, 'camera_info': info,
                  'pose_a': poses.get(self.c['target_robot']), 'pose_b': poses.get(self.c['observer_robot']),
                  'pose_a_header': headers.get(self.c['target_robot']),
                  'pose_b_header': headers.get(self.c['observer_robot']), 'detections': detections}
        return image, overlay, record, received_mono, capture_id

    def _reject(self, reason):
        self.controller.reason = reason
        if self.controller.state in ACTIVE:
            self.controller.stop(reason, 'FAULT')

    def _health(self):
        if self.c['acquisition_mode'] == 'manual':
            # This recorder never commands motion. Fresh raw poses and the
            # stillness/burst guards determine whether measurement is valid.
            self.controller.health_ok = True
            self.controller.health_reason = ''
            return
        now = time.monotonic()
        for robot in self.controller.history:
            at, state = self.robot_states.get(robot, (-math.inf, -1))
            if now-at > 1.0 or state not in (3, 4, 11):
                self.controller.health_ok = False
                self.controller.health_reason = f'{robot}: robot_state missing/stale or not READY/PAUSE/MANUALCONTROL ({state})'
                return
        self.controller.health_ok = True

    def _camera_preflight(self):
        now = time.monotonic()
        frames = []
        for side in ('left', 'right'):
            if side not in self.infos or side not in self.images or side not in self.camera_seen:
                raise ValueError(f'{side}: wait for a calibrated camera image')
            if now-self.camera_seen[side]['received_mono'] > self.c['image_max_age_sec']:
                raise ValueError(f'{side}: camera stream stopped')
            frames.append(self.images[side]['frame_id'])
        if frames[0] == frames[1]:
            raise ValueError('Left/right CameraInfo must use distinct optical frame IDs')

    def _exclusive_preflight(self):
        # The GUI's shared ROS worker can have a dormant jog publisher; multiple
        # actual cmd publishers are conservatively rejected rather than guessed.
        stamped = f"/{self.c['target_robot']}/cmd_vel_stamped"
        plain = f"/{self.c['target_robot']}/cmd_vel"
        if self.count_subscribers(stamped) == 0:
            raise ValueError('No MiR cmd_vel_stamped subscriber; start MiR hardware first')
        publishers = self.get_publishers_info_by_topic(stamped)
        others = [p for p in publishers
                  if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
        if len(publishers) != 1 or others or self.count_publishers(plain):
            raise ValueError('Other cmd_vel publishers discovered; stop/restart competing jog/controller nodes')

    def _initial_guesses(self):
        guesses = dict(self.c.get('camera_initial_guesses', {}))
        for side in ('left', 'right'):
            if side in guesses:
                continue
            try:
                t = self.tf_buffer.lookup_transform(f"{self.c['observer_robot']}/base_link",
                                                     self.images[side]['frame_id'], Time())
                p, q = t.transform.translation, t.transform.rotation
                guesses[side] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            except TransformException:
                pass
        return guesses

    def _ensure_dataset(self):
        if self.dataset is None:
            self.dataset = Dataset(self.c, self._initial_guesses())

    def _service(self, action, request, response):
        del request
        now = time.monotonic()
        active_before = self.controller.state in ACTIVE
        try:
            self._health()
            if action in ('prepare', 'verify', 'start'):
                self.controller._require_automatic()
            if action == 'prepare':
                self.controller.prepare(now, self.waypoints)
            elif action == 'verify':
                if self.controller.state != 'READY':
                    raise ValueError('Prepare and inspect a plan before the short stopping test')
                self._camera_preflight()
                self._exclusive_preflight()
                a = self.controller.poses[self.c['target_robot']][0]
                goal = a[:2, 3].copy()
                for item in self.controller.plan:
                    for point in item['path'][1:]:
                        delta = np.asarray(point)-a[:2, 3]
                        if np.linalg.norm(delta) > 0.03:
                            goal += delta/np.linalg.norm(delta)*min(0.10, np.linalg.norm(delta))
                            break
                    if np.linalg.norm(goal-a[:2, 3]) > 0.001:
                        break
                from .geometry import yaw
                self.controller.prepare(now, [[*goal, yaw(a)]])
                self.controller.start(now, verification=True)
                self.watchdog_tripped.clear()
                self.watchdog_armed.set()
            elif action == 'start':
                self._camera_preflight()
                self._exclusive_preflight()
                self.controller.start(now)
                self._ensure_dataset()
                self.watchdog_tripped.clear()
                self.watchdog_armed.set()
            elif action == 'pause':
                self.controller.pause()
                self._zero()
                self._abort_capture('Paused during burst')
            elif action == 'stop':
                self.controller.stop()
                self._zero()
                self._abort_capture('Stopped during burst')
            elif action == 'capture':
                self._camera_preflight()
                self.controller.capture(now)
                self._ensure_dataset()
                self.watchdog_tripped.clear()
                self.watchdog_armed.set()
            response.success, response.message = True, self.controller.reason
        except (ValueError, OSError) as exc:
            response.success, response.message = False, str(exc)
            # A failed action must never leave a started controller running.
            if self.controller.state in ACTIVE and not active_before:
                self.controller.stop(str(exc), 'FAULT')
                self._zero()
        self._status()
        return response

    def _waypoints(self, msg):
        if self.controller.state in ACTIVE:
            self._reject('Waypoint changes while active are forbidden')
            return
        try:
            proposed = json.loads(msg.data)
            self.controller.prepare(time.monotonic(), proposed)
            self.waypoints = proposed
        except (ValueError, TypeError) as exc:
            self.controller.stop(f'Waypoint edit rejected: {exc}', 'FAULT')
        self._status()

    def _command(self, linear, angular):
        if self.command_publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.c['target_robot']}/base_link"
        msg.twist.linear.x, msg.twist.angular.z = float(linear), float(angular)
        self.command_publisher.publish(msg)

    def _zero(self):
        self._command(0, 0)
        self.watchdog_armed.clear()

    def _guard(self):
        while not self.watchdog_stop.wait(0.025):
            now = time.monotonic()
            if self.watchdog_armed.is_set() and (
                    now-self.last_tick_mono > self.c['mocap_timeout_sec'] or
                    now-self.controller.heartbeat_at > self.c['heartbeat_timeout_sec']):
                self.watchdog_tripped.set()
                try:
                    self._command(0, 0)
                except Exception:
                    return

    def _abort_capture(self, reason):
        if self.dataset and self.dataset.pending:
            self.dataset.finish(False, reason, self.get_clock().now().nanoseconds*1e-9)
        self.capture_started = self.capture_mono = None

    def _tick(self):
        now = time.monotonic()
        self.last_tick_mono = now
        self._health()
        if self.watchdog_tripped.is_set() and self.controller.state in ACTIVE:
            self.controller.stop('Command watchdog or GUI heartbeat expired', 'FAULT')
        if self.controller.state == 'MOVING':
            try:
                self._camera_preflight()
                self._exclusive_preflight()
            except ValueError as exc:
                self.controller.stop(str(exc), 'FAULT')
        command = self.controller.tick(now)
        if self.controller.state in ACTIVE:
            self.watchdog_armed.set()
            # Re-check after tick: a concurrent watchdog zero must not be followed
            # by a stale nonzero command in this executor iteration.
            self._command(*(command if not self.watchdog_tripped.is_set() else (0, 0)))
        elif self.last_state in ACTIVE:
            self._zero()
        if self.dataset and self.dataset.pending and self.controller.state != 'CAPTURING':
            self._abort_capture(self.controller.reason)
        if self.controller.state == 'CAPTURING' and self.controller.verification_mode:
            self.controller.stop('Short test complete. Verify STOP, envelopes and native command timeout; '
                                 'then explicitly confirm boundary_verified and reload settings.')
            self._zero()
        if self.controller.state == 'CAPTURING' and self.capture_started is None:
            self._ensure_dataset()
            wall = self.get_clock().now().nanoseconds*1e-9
            self.capture_started, self.capture_mono = wall, now
            self.counts = {'left': 0, 'right': 0}
            waypoint = f'manual_{self.dataset.count}' if self.controller.manual_capture else str(self.controller.index)
            self.dataset.begin(waypoint, wall)
        while True:
            try:
                side, result, error = self.results.get_nowait()
            except queue.Empty:
                break
            self.busy[side] = False
            if error:
                self._reject(f'{side}: image decoding/detection failed: {error}')
                continue
            image, overlay, record, received_mono, capture_id = result
            self.images[side] = record
            if self.preview_publishers[side].get_subscription_count():
                preview = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                preview.header.frame_id = record['frame_id']
                preview.header.stamp = self.get_clock().now().to_msg()
                self.preview_publishers[side].publish(preview)
            if self.controller.state != 'CAPTURING' or not self.dataset.pending:
                continue
            # Only images actually exposed after the complete settlement window,
            # queued in THIS burst, with fresh raw poses at receipt can enter it.
            if capture_id != self.dataset.pending['id'] or record['stamp'] <= self.capture_started or received_mono < self.capture_mono:
                continue
            if any(record[f'pose_{letter}_header'] is None for letter in ('a', 'b')):
                self._reject('Raw pose pairing is missing; burst rejected')
                continue
            pose_ages = [record['received_at']-record[f'pose_{letter}_header']['received_at'] for letter in ('a', 'b')]
            if any(not 0 <= age <= self.c['mocap_timeout_sec'] for age in pose_ages):
                self._reject('Raw pose pairing is stale; burst rejected')
                continue
            if self.counts[side] < self.c['frames_per_camera']:
                self.dataset.image(image, record)
                self.counts[side] += 1
        if self.controller.state == 'CAPTURING' and self.capture_mono is not None:
            if now-self.capture_mono > self.c['capture_timeout_sec']:
                self.controller.stop('Camera burst timed out', 'FAULT')
            elif min(self.counts.values()) >= self.c['frames_per_camera']:
                if not self.controller.stable(now):
                    self.controller.stop('Movement at end of burst; rejected', 'FAULT')
                else:
                    self.dataset.finish(True, 'Stationary burst complete', self.get_clock().now().nanoseconds*1e-9)
                    self.capture_started = self.capture_mono = None
                    self.controller.capture_finished(now)
        if self.controller.state not in ACTIVE:
            self._abort_capture(self.controller.reason)
            if self.watchdog_armed.is_set():
                self._zero()
        if self.dataset and self.last_state != self.controller.state:
            self.dataset.event({'received_at': self.get_clock().now().nanoseconds*1e-9,
                                'state': self.controller.state, 'reason': self.controller.reason})
        self.last_state = self.controller.state

    def _status(self):
        now = time.monotonic()
        controller = self.controller
        data = {'owner_token': self.owner, 'state': controller.state, 'reason': controller.reason,
                'index': controller.index, 'plan': controller.plan, 'counts': self.counts,
                'dataset': str(self.dataset.path) if self.dataset else None,
                'motion_enabled': self.c['motion_enabled'],
                'acquisition_mode': self.c['acquisition_mode'],
                'target_robot': self.c['target_robot'], 'observer_robot': self.c['observer_robot'],
                'web_url': getattr(self, 'web_url', None),
                'measurements_saved': self.accepted_measurements(),
                'mocap': {robot: {'age_sec': now-at, 'pose': values(t)} for robot, (t, at) in controller.poses.items()},
                'cameras': {side: {key: v for key, v in info.items() if key != 'received_mono'}
                            for side, info in self.camera_seen.items()},
                'detections': {side: [d['marker'] for d in record['detections']] for side, record in self.images.items()},
                'marker_poses': {side: [{'marker': d['marker'], 'id': d['id'],
                                       'optical_frame': record['frame_id'],
                                       'pnp': d['pnp_candidates'][0] if d['pnp_candidates'] else None}
                                      for d in record['detections']] for side, record in self.images.items()}}
        self.status_publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def accepted_measurements(self):
        return self.dataset.accepted_count if self.dataset else 0

    def destroy_node(self):
        self.controller.stop('Node shutdown')
        try:
            self._zero()
        except Exception:
            pass  # The context may already have been closed by an external shutdown.
        self.watchdog_stop.set()
        self.watchdog.join(timeout=0.5)
        self.pool.shutdown(wait=True, cancel_futures=True)
        self._abort_capture('Node shutdown')
        if self.dataset:
            self.dataset.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = CalibrationSession()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
