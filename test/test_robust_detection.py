from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from match_mir_camera_calibration.calibration import redetect_measurements
from match_mir_camera_calibration.dataset import Dataset, load_dataset
from match_mir_camera_calibration.vision import Detector


@pytest.fixture
def settings():
    return yaml.safe_load((Path(__file__).parents[1]/'config/session.yaml').read_text())


INFO = {'height': 480, 'width': 640, 'distortion_model': 'plumb_bob',
        'k': [600., 0., 320., 0., 600., 240., 0., 0., 1.], 'd': [0.]*5}


def marker_image(settings, mid, errors=0, rotation=0):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, settings['dictionary']))
    generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    marker = generate(dictionary, mid, 240)
    # Corrupt whole INTERNAL cells, leaving the measured outer square intact.
    for row, col in [(2, 2), (3, 4), (5, 1)][:errors]:
        marker[30*(row+1):30*(row+2), 30*(col+1):30*(col+2)] ^= 255
    image = np.full((480, 640), 255, np.uint8)
    image[120:360, 200:440] = np.rot90(marker, rotation)
    return image


@pytest.mark.parametrize('mid', [7, 24])
@pytest.mark.parametrize('rotation', range(4))
@pytest.mark.parametrize('errors', [1, 2])
def test_known_id_recovers_code_damage_and_preserves_corner_order(settings, mid, rotation, errors):
    robust = Detector(settings)
    ordinary = Detector(dict(settings, robust_detection=False))
    damaged = marker_image(settings, mid, errors, rotation)
    assert not ordinary.detect(damaged, INFO)
    expected = ordinary.detect(marker_image(settings, mid, rotation=rotation), INFO)[0]
    found = robust.detect(damaged, INFO)
    assert len(found) == 1 and found[0]['id'] == mid
    assert found[0]['marker'] == ('front_left' if mid == 24 else 'front_right')
    assert found[0]['corrected_bits'] == errors
    assert found[0]['correction_limit_bits'] == 2
    # This also exercises the upscaled path and its pixel-centre conversion.
    np.testing.assert_allclose(found[0]['corners'], expected['corners'], atol=.2)
    np.testing.assert_allclose(found[0]['pnp_candidates'][0]['transform'][:3],
                               expected['pnp_candidates'][0]['transform'][:3], atol=.002)


@pytest.mark.parametrize('mid', [0, 1, 8, 23, 25, 100])
def test_other_family_ids_are_not_forced_to_known_ids(settings, mid):
    assert not Detector(settings).detect(marker_image(settings, mid), INFO)


def test_excessive_damage_occlusion_and_disappearance_do_not_create_measurements(settings):
    detector = Detector(settings)
    assert detector.detect(marker_image(settings, 7), INFO)
    assert not detector.detect(marker_image(settings, 7, errors=3), INFO)
    occluded = marker_image(settings, 7)
    occluded[120:165, 200:440] = 255  # Entire top edge/corners are hidden.
    assert not detector.detect(occluded, INFO)
    blank = np.full((480, 640), 255, np.uint8)
    assert not detector.detect(blank, INFO)  # No stale tracking or mocap-derived corners.


def test_duplicate_known_markers_are_rejected(settings):
    image = marker_image(settings, 7)
    image[:, :320] = np.roll(image, -130, axis=1)[:, :320]
    image[:, 320:] = np.roll(marker_image(settings, 7), 130, axis=1)[:, 320:]
    with pytest.raises(ValueError, match='Duplicate marker ID'):
        Detector(settings).detect(image, INFO)


def test_recovery_radius_also_protects_unconfigured_family_codes(settings):
    settings['dictionary'] = 'DICT_4X4_1000'
    detector = Detector(settings)
    # Selected codes 7/24 have closer neighbours elsewhere in this dictionary.
    # Restricting the list must not give them an unsafe correction radius.
    assert detector.code_distance == 2
    assert detector.recovery_dictionary.maxCorrectionBits == 0


def test_zero_false_detections_in_deterministic_random_squares(settings):
    detector = Detector(settings)
    rng = np.random.default_rng(620)
    for _ in range(32):
        cells = np.zeros((8, 8), np.uint8)
        cells[1:-1, 1:-1] = rng.integers(0, 2, (6, 6), dtype=np.uint8)*255
        image = np.full((480, 640), 255, np.uint8)
        image[120:360, 200:440] = cv2.resize(cells, (240, 240), interpolation=cv2.INTER_NEAREST)
        assert not detector.detect(image, INFO)


def test_offline_redetection_recovers_original_pixels_without_rewriting_records(settings, tmp_path):
    settings['output_dir'] = str(tmp_path)
    writer = Dataset(settings, {})
    writer.begin('test', 0.)
    damaged = marker_image(settings, 7, errors=2)
    writer.image(damaged, {'camera': 'right', 'camera_info': INFO, 'detections': []})
    writer.finish(True, 'stationary', 1.)
    writer.close()
    root, _, config, measurements = load_dataset(writer.path)
    file = root/'measurements/000000/measurement.json'
    original_file = file.read_bytes()
    original_records = deepcopy(measurements)
    updated, report = redetect_measurements(root, measurements, config)
    assert report['recorded_detections'] == 0 and report['redetected_detections'] == 1
    assert updated[0]['images'][0]['detections'][0]['id'] == 7
    assert updated[0]['images'][0]['detector']['recovery_correction_limit_bits'] == 2
    assert measurements == original_records
    assert file.read_bytes() == original_file
