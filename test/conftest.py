from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from match_mir_camera_calibration.config import validate
from match_mir_camera_calibration.geometry import transform


@pytest.fixture
def config(tmp_path):
    c = yaml.safe_load((Path(__file__).parents[1]/'config/session.yaml').read_text())
    c['acquisition_mode'] = 'automatic'
    c['dictionary'] = 'DICT_4X4_250'
    c['observer_robot'] = 'mur620b'
    c['markers'] = {'rear_left': {'id': 7, 'length_m': 0.10}, 'rear_right': {'id': 8, 'length_m': 0.10}}
    c['height_anchor']['z_m'] = 0.32
    c['bounds'] = {'x_min': -4., 'x_max': 4., 'y_min': -4., 'y_max': 4.}
    c['target_radius_m'] = c['observer_radius_m'] = 0.70
    c['output_dir'] = str(tmp_path/'sessions')
    return validate(c)


def planar(x, y=0., angle=0.):
    q = Rotation.from_euler('z', angle).as_quat()
    return transform([x, y, 0., *q])


@pytest.fixture
def enabled(config):
    c = deepcopy(config)
    for key in ('motion_enabled', 'boundary_verified', 'exclusive_control_confirmed', 'arms_stowed_confirmed'):
        c[key] = True
    return c
