"""Manual recorder with a local-network web UI, without any drive publisher."""
import argparse
from concurrent.futures import Future
import json
import os
from pathlib import Path
import queue
import secrets
import signal
import shutil
import sys
import tempfile
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .config import load
from .controller import ACTIVE
from .session_node import CalibrationSession, PREFIX
from .qr import advertised_host, qr_image, terminal_qr
from .web_server import make_server


class BrowserBridge(Node):
    def __init__(self, owner, config):
        super().__init__('mir_calibration_web')
        self.owner, self.config = owner, config
        self.lock = threading.Lock()
        self.latest, self.status_at, self.browser_at = {}, 0., 0.
        self.frames = {}
        self.actions = queue.Queue()
        self.pending = []
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, PREFIX+'/status', self._status, qos)
        for side in ('left', 'right'):
            self.create_subscription(Image, PREFIX+f'/preview/{side}',
                                     lambda msg, s=side: self._image(s, msg), qos_profile_sensor_data)
        self.heart = self.create_publisher(String, PREFIX+'/heartbeat', 1)
        self.service_clients = {action: self.create_client(Trigger, PREFIX+'/'+action) for action in ('capture', 'stop')}
        self.create_timer(.10, self._poll)

    def _status(self, msg):
        data = json.loads(msg.data)
        if data.get('owner_token') == self.owner:
            with self.lock:
                self.latest, self.status_at = data, time.monotonic()

    def _image(self, side, msg):
        pixels = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            with self.lock:
                self.frames[side] = (encoded.tobytes(), time.monotonic())

    def heartbeat(self):
        with self.lock:
            self.browser_at = time.monotonic()

    def jpeg(self, side):
        with self.lock:
            value = self.frames.get(side)
        if value and time.monotonic()-value[1] <= self.config['image_max_age_sec']:
            return value[0]
        return None

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            data = dict(self.latest)
            delay = now-self.status_at
            frame_times = {side: at for side, (_, at) in self.frames.items()}
        data.pop('owner_token', None)
        data['backend_live'] = delay < 1.
        data['camera_live'] = {side: side in frame_times and now-frame_times[side] <= self.config['image_max_age_sec']
                               for side in ('left', 'right')}
        mocap = data.get('mocap', {})
        data['mocap_live'] = {robot: robot in mocap and mocap[robot]['age_sec']+delay <= self.config['mocap_timeout_sec']
                              for robot in (self.config['target_robot'], self.config['observer_robot'])}
        camera_sources = data.get('cameras', {})
        wall = self.get_clock().now().nanoseconds*1e-9
        sources_live = all(side in camera_sources and
                           -.05 <= wall-camera_sources[side]['source_stamp'] <= self.config['image_max_age_sec']
                           for side in ('left', 'right'))
        data['can_capture'] = (data['backend_live'] and data.get('state') not in ACTIVE and
                               all(data['camera_live'].values()) and all(data['mocap_live'].values()) and sources_live)
        return data

    def request(self, action):
        future = Future()
        if action not in self.service_clients:
            future.set_result({'success': False, 'message': 'Fahraktionen sind deaktiviert.'})
        elif action == 'capture' and not self.snapshot()['can_capture']:
            future.set_result({'success': False, 'message': 'Aufnahme läuft bereits oder Kamera/Mocap ist nicht frisch.'})
        else:
            self.actions.put((action, future, time.monotonic()+4.))
        return future

    def _poll(self):
        now = time.monotonic()
        with self.lock:
            browser_at = self.browser_at
        if now-browser_at < .6:
            self.heart.publish(String(data=self.owner))
        while not self.actions.empty():
            action, result, deadline = self.actions.get_nowait()
            if now > deadline or not self.service_clients[action].service_is_ready():
                result.set_result({'success': False, 'message': 'Aufnahmedienst ist nicht erreichbar.'})
                continue
            self.pending.append((self.service_clients[action].call_async(Trigger.Request()), result, deadline))
        for pending in list(self.pending):
            ros_future, result, deadline = pending
            if ros_future.done():
                try:
                    response = ros_future.result()
                    result.set_result({'success': response.success, 'message': response.message})
                except Exception as exc:
                    result.set_exception(exc)
                self.pending.remove(pending)
            elif now > deadline:
                ros_future.cancel()
                result.set_result({'success': False, 'message': 'Antwort ausgeblieben; Sitzungsstatus prüfen.'})
                self.pending.remove(pending)


def main(args=None):
    parser = argparse.ArgumentParser(description='Manuelle MuR-Kalibrierung mit iPhone-Webansicht')
    parser.add_argument('--config', default=str(Path(get_package_share_directory('match_mir_camera_calibration'))/'config/session.yaml'))
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--advertise-host', help='WLAN/LAN-IP oder Rechnername für den iPhone-QR-Code')
    parser.add_argument('--owner-token', default=uuid.uuid4().hex)
    options, ros_args = parser.parse_known_args(args)
    config = load(options.config)
    if config['acquisition_mode'] != 'manual':
        parser.error('Der Web-Aufnahmemodus erfordert acquisition_mode: manual.')
    os.environ.setdefault('ROS_DOMAIN_ID', '62')
    os.environ.setdefault('ROS_STATIC_PEERS', f"{config['target_robot']};{config['observer_robot']}")
    rclpy.init(args=ros_args+['--ros-args', '-p', f'config_path:={options.config}', '-p', f'owner_token:={options.owner_token}'],
                signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    session = bridge = server = executor = probe = None
    qr_directory = None
    try:
        # Prevent two backends from responding to the same capture service.
        probe = rclpy.create_node('mir_calibration_web_startup')
        client = probe.create_client(Trigger, PREFIX+'/capture')
        if client.wait_for_service(timeout_sec=1.):
            raise RuntimeError('Ein Kalibrierungsbackend läuft bereits. Vor dem Webstart beenden.')
        probe.destroy_node()
        probe = None
        session = CalibrationSession()
        bridge = BrowserBridge(options.owner_token, config)
        token = secrets.token_urlsafe(24)
        server = make_server(options.host, options.port, bridge, token)
        threading.Thread(target=server.serve_forever, daemon=True, name='calibration-http').start()
        port = server.server_address[1]
        host = options.advertise_host or advertised_host(options.host, config['observer_robot'])
        url = f'http://{host}:{port}/?token={token}'
        session.web_url = url
        qr_directory = Path(tempfile.mkdtemp(prefix='mir_calibration_qr_'))
        image_path = qr_directory/'iphone.png'
        if not cv2.imwrite(str(image_path), qr_image(url)):
            raise OSError('QR-Code konnte nicht gespeichert werden.')
        print('\niPhone-Webansicht: QR-Code mit der iPhone-Kamera scannen.', flush=True)
        if sys.stdout.isatty():
            print(terminal_qr(url), flush=True)
        print(f'QR-Code als Bild: {image_path}\n'
              f'Bild öffnen: xdg-open {image_path}\n'
              'In der GUI: „iPhone-QR-Code“ drücken.\n'
              f'Link als Alternative: {url}\n'
              'Manueller Modus: keine Fahrbefehle; neue Pose im Browser bestätigen.\n', flush=True)
        executor = SingleThreadedExecutor()
        executor.add_node(session)
        executor.add_node(bridge)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if executor:
            executor.shutdown()
        for node in (bridge, session, probe):
            if node:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if qr_directory:
            shutil.rmtree(qr_directory)


if __name__ == '__main__':
    main()
