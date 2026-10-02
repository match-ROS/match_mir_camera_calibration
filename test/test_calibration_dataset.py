from copy import deepcopy

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from match_mir_camera_calibration.calibration import (
    CalibrationError, JointProblem, check_coverage, fit, solve_dataset,
)
from match_mir_camera_calibration.dataset import Dataset, load_dataset
from match_mir_camera_calibration.geometry import distance, inverse, marker_points, pack, transform, values
from match_mir_camera_calibration.vision import Detector, camera_model, hypotheses
from conftest import planar


def synthetic(config, noise=0.0, only_translation=False):
    rc = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    rm = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
    ts = {}
    for side, y in [('left', .25), ('right', -.25)]:
        ts[side] = transform([.55, y, .25, *Rotation.from_matrix(rc).as_quat()])
    for marker, y in [('rear_left', .30), ('rear_right', -.30)]:
        ts[marker] = transform([-.60, y, .32, *Rotation.from_matrix(rm).as_quat()])
    k = np.array([[600., 0, 640], [0, 600., 360], [0, 0, 1]])
    info = {'height': 720, 'width': 1280, 'distortion_model': 'plumb_bob',
            'k': k.ravel().tolist(), 'd': [0., 0., 0., 0., 0.]}
    obs, records = [], []
    rng = np.random.default_rng(42)
    index = 0
    for x in [1.7, 2.1, 2.5]:
        for y in [-.35, 0., .35]:
            for angle in ([0.] if only_translation else [-.30, 0., .30]):
                a, b = planar(x, y, angle), planar(0)
                images = []
                mid = f'{index:06d}'
                for side in ('left', 'right'):
                    detections = []
                    for marker in ('rear_left', 'rear_right'):
                        t = inverse(ts[side]) @ a @ ts[marker]
                        pts = marker_points(config['markers'][marker]['length_m'])
                        corners = cv2.projectPoints(pts, cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], k, np.zeros(5))[0].reshape(4, 2)
                        corners += rng.normal(size=(4, 2))*noise
                        candidates = hypotheses(corners, .1, k, np.zeros(5))
                        obs.append({'side': side, 'marker': marker, 'relative': a, 'corners': corners,
                                    'k': k, 'd': np.zeros(5), 'waypoint': mid, 'candidates': candidates})
                        detections.append({'marker': marker, 'id': config['markers'][marker]['id'],
                                           'corners': corners.tolist(), 'pnp_candidates': candidates})
                    images.append({'camera': side, 'frame_id': f'mur620b/{side}_optical',
                                   'camera_info': info, 'pose_a': values(a), 'pose_b': values(b),
                                   'detections': detections})
                records.append({'id': mid, 'images': images, 'accepted': True})
                index += 1
    guesses = {side: values(ts[side] @ planar(.01, -.01, .02)) for side in ('left', 'right')}
    return ts, obs, guesses, records


def test_transform_direction_and_height_anchor(config):
    ts, obs, guesses, _ = synthetic(config)
    problem, x, report = fit(config, obs, guesses, starts=1)
    assert report['rank'] == 23
    fitted = problem.transforms(x)
    for name in ts:
        d, angle = distance(ts[name], fitted[name])
        assert d < 1e-5, (name, d)
        assert angle < 1e-5, (name, angle)
    assert fitted['rear_left'][2, 3] == config['height_anchor']['z_m']


def test_noisy_reconstruction(config):
    ts, obs, guesses, _ = synthetic(config, noise=.15)
    problem, x, report = fit(config, obs, guesses, starts=1)
    for name, fitted in problem.transforms(x).items():
        d, angle = distance(ts[name], fitted)
        assert d < .01
        assert np.degrees(angle) < 1.0
    assert report['training']['corner_error_px']['rms'] < .4


def test_disconnected_observations_are_rejected(config):
    _, obs, _, _ = synthetic(config)
    disconnected = [o for o in obs if (o['side'], o['marker']) in [('left', 'rear_left'), ('right', 'rear_right')]]
    with pytest.raises(CalibrationError, match='Disconnected'):
        check_coverage(disconnected)


def test_translation_only_motion_is_underdetermined(config):
    _, obs, guesses, _ = synthetic(config, only_translation=True)
    with pytest.raises(CalibrationError, match='rank'):
        fit(config, obs, guesses, starts=1)


def test_solver_needs_camera_initial_guesses(config):
    _, obs, _, _ = synthetic(config)
    with pytest.raises(CalibrationError, match='initial camera guess'):
        fit(config, obs, {}, starts=1)


def test_dataset_roundtrip_rejected_bursts_excluded_and_holdout_by_pose(config):
    ts, _, guesses, records = synthetic(config)
    writer = Dataset(config, guesses)
    for i, record in enumerate(records):
        writer.begin(str(i), float(i))
        for image in record['images']:
            writer.image(np.zeros((720, 1280, 3), np.uint8), image)
        writer.finish(True, 'test', float(i)+.5)
    writer.begin('rejected', 100.)
    writer.image(np.zeros((10, 10, 3), np.uint8), dict(records[0]['images'][0]))
    writer.finish(False, 'Movement during capture', 101.)
    writer.close()
    root, _, _, accepted = load_dataset(writer.path)
    assert len(accepted) == 27
    pixels = cv2.imread(str(root/accepted[0]['images'][0]['path']))
    assert pixels.shape == (720, 1280, 3)
    export = solve_dataset(root, starts=1)
    assert export['quality'] == 'validated'
    assert export['report']['validation']['detections'] == 20
    assert export['report']['validation_pose_ids'] == ['000004', '000009', '000014', '000019', '000024']
    assert export['transforms']['left']['parent_frame'] == 'mur620b/base_link'
    assert export['transforms']['rear_left']['child_frame'] == 'mur620a/aruco_rear_left'
    assert (root/'calibration.yaml').is_file()


def test_actual_aruco_detection_and_lossless_camera_model(config):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    marker = cv2.aruco.drawMarker(dictionary, 7, 180) if hasattr(cv2.aruco, 'drawMarker') else cv2.aruco.generateImageMarker(dictionary, 7, 180)
    image = np.full((480, 640), 255, np.uint8)
    image[140:320, 230:410] = marker
    info = {'height': 480, 'width': 640, 'distortion_model': 'plumb_bob',
            'k': [600., 0, 320., 0, 600., 240., 0, 0, 1.], 'd': [0.]*5}
    found = Detector(config).detect(image, info)
    assert len(found) == 1 and found[0]['marker'] == 'rear_left'
    assert found[0]['pnp_candidates']
    assert found[0]['pnp_candidates'][0]['reprojection_px'] < .5
    info['binning_x'] = 2
    with pytest.raises(ValueError, match='Binned'):
        camera_model(info)
