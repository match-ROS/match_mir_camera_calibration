import math

import numpy as np
import pytest

from match_mir_camera_calibration.config import ConfigurationError, validate
from match_mir_camera_calibration.controller import SessionController
from match_mir_camera_calibration.planner import Planner, PlanningError
from conftest import planar


def observed(controller, at, a=None, b=None):
    controller.observe(controller.c['target_robot'], planar(2) if a is None else a, at)
    controller.observe(controller.c['observer_robot'], planar(0) if b is None else b, at)
    controller.heartbeat_at = at
    controller.health_ok = True


def test_required_values_and_distinct_marker_ids(config):
    config['markers']['rear_left']['length_m'] = None
    with pytest.raises(ConfigurationError, match='finite number'):
        validate(config)
    config['markers']['rear_left']['length_m'] = .1
    config['markers']['rear_right']['id'] = 7
    with pytest.raises(ConfigurationError, match='different'):
        validate(config)


def test_planner_routes_around_b_and_keeps_full_envelope(config):
    p = Planner(config, [0, 0])
    path = p.route([-2.5, 0], [2.5, 0])
    assert len(path) > 2
    assert all(p.segment_safe(a, b) for a, b in zip(path, path[1:]))
    assert not p.safe([3.5, 0])
    assert not p.safe([1, 0])
    assert p.reserve > config['margin_m']
    with pytest.raises(PlanningError):
        p.route([1, 0], [2.5, 0])


def test_raster_faces_rear_toward_b_and_excludes_collision(config):
    p = Planner(config, [0, 0])
    raster = p.raster()
    assert len(raster) == 24  # Centre position is inside B's envelope.
    assert all(p.safe(pose[:2]) for pose in raster)
    for x, y, angle in raster[1::3]:
        assert abs(math.atan2(math.sin(angle-math.atan2(y, x)), math.cos(angle-math.atan2(y, x)))) < 1e-8


def test_motion_is_locked_by_default(config):
    c = SessionController(config)
    observed(c, 0)
    c.prepare(0, [[2.2, 0., 0.]])
    assert c.tick(0.1) == (0., 0.)
    with pytest.raises(ValueError, match='motion_enabled'):
        c.start(0.1)


def test_front_marker_raster_faces_front_toward_observer(config):
    config['markers'] = {name.replace('rear_', 'front_'): item for name, item in config['markers'].items()}
    config['height_anchor']['marker'] = 'front_left'
    config = validate(config)
    planner = Planner(config, [0, 0])
    for x, y, angle in planner.raster()[1::3]:
        direction = np.array([math.cos(angle), math.sin(angle)])
        np.testing.assert_allclose(direction, -np.array([x, y])/math.hypot(x, y), atol=1e-8)
        assert planner.safe([x, y])


def test_short_verification_is_slow_and_does_not_unlock_raster(enabled):
    enabled['boundary_verified'] = False
    c = SessionController(enabled)
    observed(c, 0)
    c.prepare(0, [[2.1, 0., 0.]])
    with pytest.raises(ValueError, match='boundary_verified'):
        c.start(0)
    c.start(0, verification=True)
    for at in np.arange(.01, .51, .01):
        observed(c, at)
        assert 0 <= c.tick(at)[0] <= .01
    c.stop()
    assert not c.verification_mode
    observed(c, .51)
    c.prepare(.51, [[2.1, 0., 0.]])
    with pytest.raises(ValueError, match='boundary_verified'):
        c.start(.51)


@pytest.mark.parametrize('failure', ['mocap', 'heartbeat', 'observer', 'bounds', 'robot_state'])
def test_each_guard_stops_and_requires_explicit_restart(enabled, failure):
    c = SessionController(enabled)
    observed(c, 0)
    c.prepare(0, [[2.2, 0., 0.]])
    c.start(0)
    observed(c, 0.1)
    assert c.tick(0.1)[0] > 0
    if failure == 'mocap':
        now = 0.4
        c.heartbeat_at = now
    else:
        now = 0.2
        observed(c, now, a=planar(3.8) if failure == 'bounds' else None,
                 b=planar(0.1) if failure == 'observer' else None)
        if failure == 'heartbeat':
            c.heartbeat_at = -1
        if failure == 'robot_state':
            c.health_ok, c.health_reason = False, 'emergency stop'
    assert c.tick(now) == (0., 0.)
    assert c.state == 'FAULT'
    observed(c, now+0.01)
    assert c.tick(now+0.01) == (0., 0.)
    with pytest.raises(ValueError, match='Prepare'):
        c.start(now+0.01)


def test_pause_resume_and_two_second_settlement(enabled):
    c = SessionController(enabled)
    observed(c, 0)
    c.prepare(0, [[2., 0., 0.]])
    c.start(0)
    c.tick(0)
    assert c.state == 'SETTLING'
    c.pause()
    assert c.state == 'PAUSED'
    assert c.tick(.01) == (0., 0.)
    observed(c, .02)
    c.start(.02)
    for at in np.arange(.03, 1.99, .01):
        observed(c, at)
        c.tick(at)
        assert c.state != 'CAPTURING'
    observed(c, 2.1)
    c.tick(2.1)
    assert c.state == 'CAPTURING'
    observed(c, 2.11, a=planar(2.01))
    assert c.tick(2.11) == (0., 0.)
    assert c.state == 'FAULT'


def test_stationary_capture_works_with_motion_locked(config):
    c = SessionController(config)
    observed(c, 0)
    c.capture(0)
    for at in np.arange(.01, 2.05, .01):
        observed(c, at)
        assert c.tick(at) == (0., 0.)
    assert c.state == 'CAPTURING'
    c.capture_finished(2.05)
    assert c.state == 'STOPPED'


def test_settlement_rejects_pose_gaps(enabled):
    c = SessionController(enabled)
    observed(c, 0)
    c.capture(0)
    observed(c, 2.01)
    assert not c.stable(2.01)


def test_complete_simulated_raster_drives_settles_and_finishes(enabled):
    c = SessionController(enabled)
    a, b, at = planar(2, 0, 0), planar(0), 0.
    observed(c, at, a, b)
    c.prepare(at, [[2.08, 0., .06], [2.08, .08, 0.]])
    c.start(at)
    captures = 0
    for _ in range(15000):
        at += .05
        observed(c, at, a, b)
        linear, angular = c.tick(at)
        angle = math.atan2(a[1, 0], a[0, 0])
        a = planar(a[0, 3]+linear*math.cos(angle)*.05,
                   a[1, 3]+linear*math.sin(angle)*.05, angle+angular*.05)
        if c.state == 'CAPTURING':
            captures += 1
            c.capture_finished(at)
        if c.state in ('COMPLETED', 'FAULT'):
            break
    assert c.state == 'COMPLETED', c.reason
    assert captures == 2
