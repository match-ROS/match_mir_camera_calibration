"""Validated session settings shared by GUI, recorder and offline solver."""
from copy import deepcopy
from pathlib import Path
import math
import re

import cv2
import yaml


class ConfigurationError(ValueError):
    pass


def number(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigurationError(f'{name}: a finite number is required')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ConfigurationError(f'{name}: outside allowed range [{low}, {high}]')
    return float(value)


def validate(raw):
    if not isinstance(raw, dict):
        raise ConfigurationError('Configuration must be a YAML mapping')
    c = deepcopy(raw)
    try:
        mode = c.setdefault('acquisition_mode', 'automatic')
        if mode not in ('manual', 'automatic'):
            raise ConfigurationError('acquisition_mode must be manual or automatic')
        if mode == 'manual' and c['motion_enabled']:
            raise ConfigurationError('Manual acquisition requires motion_enabled: false')
        if not isinstance(c.setdefault('preview_rotate_ccw', False), bool):
            raise ConfigurationError('preview_rotate_ccw must be boolean')
        for key in ('target_robot', 'observer_robot'):
            if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', str(c[key])):
                raise ConfigurationError(f'{key}: invalid robot namespace')
        if c['target_robot'] == c['observer_robot']:
            raise ConfigurationError('Target and observer must be different robots')
        dictionary = c['dictionary']
        if not isinstance(dictionary, str) or not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ConfigurationError('Unknown ArUco dictionary')
        capacity = len(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)).bytesList)
        if set(c['markers']) != {'rear_left', 'rear_right'}:
            raise ConfigurationError('Configure rear_left and rear_right markers')
        ids = []
        for name, marker in c['markers'].items():
            mid = marker['id']
            if isinstance(mid, bool) or not isinstance(mid, int) or not 0 <= mid < capacity:
                raise ConfigurationError(f'{name}: id must be an integer in [0, {capacity - 1}]')
            marker['length_m'] = number(marker['length_m'], f'{name}.length_m', 0.001, 1.0)
            ids.append(mid)
        if len(set(ids)) != 2:
            raise ConfigurationError('Marker IDs must be different')
        if c['height_anchor']['marker'] not in c['markers']:
            raise ConfigurationError('height_anchor.marker must identify a configured marker')
        number(c['height_anchor']['z_m'], 'height_anchor.z_m', -1.0, 3.0)
        b = c['bounds']
        optional_bounds = mode == 'manual' and all(b[key] is None for key in ('x_min', 'x_max', 'y_min', 'y_max'))
        for key in ('x_min', 'x_max', 'y_min', 'y_max'):
            if not optional_bounds:
                b[key] = number(b[key], f'bounds.{key}')
        if not optional_bounds and (b['x_min'] >= b['x_max'] or b['y_min'] >= b['y_max']):
            raise ConfigurationError('Bounds must describe a nonempty rectangle')
        limits = {
            'target_radius_m': (0.1, 5.0), 'observer_radius_m': (0.1, 5.0),
            'margin_m': (0.1, 5.0), 'planner_resolution_m': (0.02, 0.5),
            'max_linear_mps': (0.001, 0.05), 'max_angular_rps': (0.001, 0.10),
            'linear_acceleration_mps2': (0.005, 0.05),
            'angular_acceleration_rps2': (0.01, 0.10),
            'reaction_time_sec': (0.5, 5.0), 'mocap_timeout_sec': (0.05, 0.2),
            'heartbeat_timeout_sec': (0.25, 0.75), 'image_max_age_sec': (0.05, 1.0),
            'settle_sec': (2.0, 20.0), 'still_translation_m': (0.0001, 0.01),
            'still_rotation_deg': (0.01, 1.0), 'position_tolerance_m': (0.002, 0.03),
            'yaw_tolerance_deg': (0.1, 2.0), 'observer_translation_tolerance_m': (0.003, 0.03),
            'observer_rotation_tolerance_deg': (0.1, 2.0),
            'capture_timeout_sec': (5.0, 120.0), 'segment_timeout_sec': (10.0, 600.0),
        }
        for key, (low, high) in limits.items():
            if mode == 'manual' and key in ('target_radius_m', 'observer_radius_m') and c[key] is None:
                continue
            c[key] = number(c[key], key, low, high)
        for key in ('motion_enabled', 'boundary_verified', 'exclusive_control_confirmed', 'arms_stowed_confirmed'):
            if not isinstance(c[key], bool):
                raise ConfigurationError(f'{key}: boolean required')
        for key in ('nx', 'ny'):
            n = c['grid'][key]
            if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 12:
                raise ConfigurationError(f'grid.{key}: integer from 1 to 12 required')
        angles = c['grid']['yaw_offsets_deg']
        if not isinstance(angles, list) or not 1 <= len(angles) <= 9:
            raise ConfigurationError('grid.yaw_offsets_deg: 1 to 9 angles required')
        for angle in angles:
            number(angle, 'yaw offset', -70, 70)
        n = c['frames_per_camera']
        if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 30:
            raise ConfigurationError('frames_per_camera: integer from 1 to 30 required')
        if not isinstance(c['output_dir'], str) or not c['output_dir'].strip():
            raise ConfigurationError('output_dir is required')
        for side, guess in c.get('camera_initial_guesses', {}).items():
            if side not in ('left', 'right') or len(guess) != 7:
                raise ConfigurationError('Camera guesses: left/right with [xyz, quaternion xyzw]')
            for value in guess:
                number(value, f'{side} initial guess')
            if sum(v*v for v in guess[3:]) < 1e-8:
                raise ConfigurationError('Initial quaternion must be nonzero')
    except (KeyError, TypeError) as exc:
        raise ConfigurationError(f'Missing or malformed setting: {exc}') from exc
    return c


def load(path):
    return validate(yaml.safe_load(Path(path).expanduser().read_text()))
