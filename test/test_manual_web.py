from concurrent.futures import Future
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest
import yaml

from match_mir_camera_calibration.config import ConfigurationError, validate
from match_mir_camera_calibration.controller import SessionController
from match_mir_camera_calibration.vision import Detector
from match_mir_camera_calibration.web_server import make_server
from conftest import planar


def manual_settings():
    return validate(yaml.safe_load((Path(__file__).parents[1]/'config/session.yaml').read_text()))


def observe(controller, at, a=2., b=0.):
    controller.observe(controller.c['target_robot'], planar(a), at)
    controller.observe(controller.c['observer_robot'], planar(b), at)
    controller.heartbeat_at = at
    controller.health_ok = True


def test_manual_defaults_capture_without_planner_and_reject_motion():
    c = manual_settings()
    assert c['observer_robot'] == 'mur620d'
    assert c['bounds']['x_min'] is None and c['target_radius_m'] is None
    controller = SessionController(c)
    observe(controller, 0.)
    for action in (controller.prepare, controller.start):
        with pytest.raises(ValueError, match='Manual acquisition'):
            action(0.)
    controller.capture(0.)
    for at in np.arange(.01, 2.1, .01):
        observe(controller, at)
        assert controller.tick(at) == (0., 0.)
    assert controller.state == 'CAPTURING'
    controller.capture_finished(2.1)
    assert controller.state == 'STOPPED'
    # Joystick repositioning after an accepted measurement is allowed.
    observe(controller, 2.2, a=3.)
    controller.capture(2.2)
    for at in np.arange(2.21, 4.5, .01):
        observe(controller, at, a=3.)
        controller.tick(at)
    assert controller.state == 'CAPTURING'
    observe(controller, 4.51, a=3.02)
    controller.tick(4.51)
    assert controller.state == 'FAULT' and 'Movement' in controller.reason
    c['motion_enabled'] = True
    with pytest.raises(ConfigurationError, match='motion_enabled'):
        validate(c)


@pytest.mark.parametrize('failure', ['mocap', 'browser', 'observer'])
def test_manual_capture_rejects_reference_and_connection_loss(failure):
    controller = SessionController(manual_settings())
    observe(controller, 0.)
    controller.capture(0.)
    observe(controller, .1, b=.1 if failure == 'observer' else 0.)
    if failure == 'browser':
        controller.heartbeat_at = -1.
    controller.tick(.4 if failure == 'mocap' else .1)
    assert controller.state == 'FAULT'
    assert controller.tick(.5) == (0., 0.)


def test_apriltag_detection_and_upright_pose_overlay():
    c = manual_settings()
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, c['dictionary']))
    image = np.full((480, 640, 3), 255, np.uint8)
    generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    image[140:320, 50:230] = generate(dictionary, 0, 180)[..., None]
    image[140:320, 390:570] = generate(dictionary, 1, 180)[..., None]
    info = {'height': 480, 'width': 640, 'distortion_model': 'plumb_bob',
            'k': [600., 0., 320., 0., 600., 240., 0., 0., 1.], 'd': [0.]*5}
    detector = Detector(c)
    detections = detector.detect(image, info)
    assert {d['id']: d['marker'] for d in detections} == {0: 'rear_left', 1: 'rear_right'}
    assert all(d['pnp_candidates'][0]['transform'][2] > .4 for d in detections)
    original = image.copy()
    overlay = detector.overlay(image, detections, info, rotate_ccw=True)
    assert overlay.shape == (640, 480, 3)
    assert np.array_equal(image, original)  # Lossless measurement pixels stay untouched.
    assert not np.array_equal(overlay, cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE))


class FakeBridge:
    def __init__(self):
        self.actions = []
        self.pulses = 0
        self.frame = b'jpeg-frame'
    def snapshot(self):
        return {'state': 'STOPPED', 'can_capture': True, 'measurements_saved': 2}
    def jpeg(self, side):
        return self.frame
    def heartbeat(self):
        self.pulses += 1
    def request(self, action):
        self.actions.append(action)
        future = Future()
        future.set_result({'success': True, 'message': 'ok'})
        return future


def test_phone_http_ui_capture_cancel_auth_and_stale_images():
    bridge = FakeBridge()
    server = make_server('127.0.0.1', 0, bridge, 'test-token')
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    def get(path, method='GET', authorized=True):
        req = Request(base+path, method=method,
                      headers={'X-Session-Token': 'test-token'} if authorized else {})
        return urlopen(req, timeout=2.)
    try:
        with pytest.raises(HTTPError) as error:
            get('/api/capture', 'POST', authorized=False)
        assert error.value.code == 403 and not bridge.actions
        assert b'Neue Pose aufnehmen' in get('/?token=test-token', authorized=False).read()
        assert json.loads(get('/api/status').read())['measurements_saved'] == 2
        assert get('/image/left.jpg').read() == bridge.frame
        assert json.loads(get('/api/heartbeat', 'POST').read())['success']
        assert bridge.pulses == 1
        for path in ('capture', 'cancel'):
            assert json.loads(get('/api/'+path, 'POST').read())['success']
        assert bridge.actions == ['capture', 'stop']
        for path in ('start', 'verify', 'prepare'):
            with pytest.raises(HTTPError) as error:
                get('/api/'+path, 'POST')
            assert error.value.code == 404
        bridge.frame = None
        with pytest.raises(HTTPError) as error:
            get('/image/left.jpg')
        assert error.value.code == 503
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
