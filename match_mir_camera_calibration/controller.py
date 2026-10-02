"""ROS-independent, fail-closed motion and stop-and-capture state machine."""
from collections import deque
import math

import numpy as np

from .geometry import distance, wrap, yaw
from .planner import Planner


ACTIVE = {'MOVING', 'SETTLING', 'CAPTURING'}


class SessionController:
    def __init__(self, config):
        self.c = config
        self.state = 'IDLE'
        self.reason = 'Prepare a plan or request a stationary capture'
        self.poses = {}
        self.history = {config['target_robot']: deque(), config['observer_robot']: deque()}
        self.heartbeat_at = -math.inf
        self.health_ok = False
        self.health_reason = 'Robot state not received'
        self.plan = []
        self.planner = None
        self.observer_reference = None
        self.ready_reference = None
        self.index = 0
        self.segment = 1
        self.segment_started = 0.0
        self.last_tick = None
        self.command = (0.0, 0.0)
        self.capture_ready = False
        self.manual_capture = False
        self.verification_mode = False

    def observe(self, robot, t, now):
        if robot not in self.history:
            return
        self.poses[robot] = (np.array(t, copy=True), now)
        history = self.history[robot]
        history.append((now, np.array(t, copy=True)))
        while history and now - history[0][0] > self.c['settle_sec'] + 1.0:
            history.popleft()

    def _fresh(self, now):
        for robot in self.history:
            if robot not in self.poses or not 0 <= now-self.poses[robot][1] <= self.c['mocap_timeout_sec']:
                raise ValueError(f'Raw mocap stale or missing: {robot}')

    def prepare(self, now, waypoints=None):
        if self.state in ACTIVE:
            raise ValueError('Pause or stop before changing the plan')
        self._fresh(now)
        a = self.poses[self.c['target_robot']][0]
        b = self.poses[self.c['observer_robot']][0]
        self.planner = Planner(self.c, b[:2, 3])
        self.plan = self.planner.plan(a[:2, 3], waypoints)
        self.observer_reference = b.copy()
        self.ready_reference = a.copy()
        self.index, self.segment = 0, 1
        self.state, self.reason = 'READY', 'Preview ready; motion has not started'
        self.manual_capture = False

    def _safety(self, now, require_motion=False):
        self._fresh(now)
        if not 0 <= now-self.heartbeat_at <= self.c['heartbeat_timeout_sec']:
            raise ValueError('GUI heartbeat lost')
        if not self.health_ok:
            raise ValueError(self.health_reason)
        if require_motion:
            for key in ('motion_enabled', 'boundary_verified', 'exclusive_control_confirmed', 'arms_stowed_confirmed'):
                if key == 'boundary_verified' and self.verification_mode:
                    continue
                if not self.c[key]:
                    raise ValueError(f'Motion locked: {key} is false')
        a = self.poses[self.c['target_robot']][0]
        b = self.poses[self.c['observer_robot']][0]
        if not self.planner or not self.planner.safe(a[:2, 3]):
            raise ValueError('Target envelope violates safe area or observer distance')
        translation, rotation = distance(b, self.observer_reference)
        if translation > self.c['observer_translation_tolerance_m'] or rotation > math.radians(self.c['observer_rotation_tolerance_deg']):
            raise ValueError('Observer moved; prepare a new plan')

    def start(self, now, verification=False):
        if self.state not in {'READY', 'PAUSED'} or not self.plan:
            raise ValueError('Prepare a plan before starting')
        self.verification_mode = verification
        try:
            self._safety(now, require_motion=True)
        except ValueError:
            self.verification_mode = False
            raise
        a = self.poses[self.c['target_robot']][0]
        if self.ready_reference is not None and distance(a, self.ready_reference)[0] > self.c['position_tolerance_m']:
            raise ValueError('Target moved since preview/pause; prepare a new plan')
        self.manual_capture = False
        self.state = 'MOVING'
        self.reason = 'Driving to measurement pose'
        self.segment_started = now
        self.last_tick = now
        self.capture_ready = False

    def capture(self, now):
        if self.state in ACTIVE:
            raise ValueError('A session is already active')
        self._fresh(now)
        a = self.poses[self.c['target_robot']][0]
        b = self.poses[self.c['observer_robot']][0]
        self.planner = Planner(self.c, b[:2, 3])
        self.observer_reference = b.copy()
        self._safety(now)
        self.manual_capture = True
        self.verification_mode = False
        self.state, self.reason = 'SETTLING', 'Waiting for stationary capture'
        self.segment_started = now
        self.capture_ready = False

    def stop(self, reason='Stopped by user', state='STOPPED'):
        self.state, self.reason = state, reason
        self.command = (0.0, 0.0)
        self.capture_ready = False
        self.verification_mode = False
        if self.c['target_robot'] in self.poses:
            self.ready_reference = self.poses[self.c['target_robot']][0].copy()

    def pause(self):
        if self.state not in ACTIVE:
            raise ValueError('No active session to pause')
        self.stop('Paused; explicit Start required to resume', 'PAUSED')

    def stable(self, now, duration=None):
        duration = self.c['settle_sec'] if duration is None else duration
        for history in self.history.values():
            samples = [(at, t) for at, t in history if at >= now-duration-self.c['mocap_timeout_sec']]
            if not samples or samples[0][0] > now-duration:
                return False
            for (previous_at, _), (at, _) in zip(samples, samples[1:]):
                if at-previous_at > self.c['mocap_timeout_sec']:
                    return False
            reference = samples[0][1]
            for _, t in samples:
                d, angle = distance(reference, t)
                if d > self.c['still_translation_m'] or angle > math.radians(self.c['still_rotation_deg']):
                    return False
        return True

    def tick(self, now):
        dt = min(max(now-(self.last_tick if self.last_tick is not None else now), 0.0), 0.1)
        self.last_tick = now
        if self.state not in ACTIVE:
            self.command = (0.0, 0.0)
            return self.command
        try:
            self._safety(now, require_motion=not self.manual_capture)
            if now-self.segment_started > self.c['segment_timeout_sec']:
                raise ValueError('Motion/settling timed out')
        except ValueError as exc:
            self.stop(str(exc), 'FAULT')
            return self.command
        if self.state in {'SETTLING', 'CAPTURING'}:
            self.command = (0.0, 0.0)
            if self.state == 'SETTLING' and self.stable(now):
                self.state, self.reason = 'CAPTURING', 'Recording new camera frames'
                self.capture_ready = True
            elif self.state == 'CAPTURING' and not self.stable(now):
                self.stop('Movement during burst; measurement rejected', 'FAULT')
            return self.command
        item = self.plan[self.index]
        a = self.poses[self.c['target_robot']][0]
        path = item['path']
        # Waypoints are rotations in place followed by a straight guarded segment.
        while self.segment < len(path) and np.linalg.norm(np.asarray(path[self.segment])-a[:2, 3]) <= self.c['position_tolerance_m']:
            self.segment += 1
            self.segment_started = now
        linear, angular = 0.0, 0.0
        max_linear = min(self.c['max_linear_mps'], 0.01) if self.verification_mode else self.c['max_linear_mps']
        max_angular = min(self.c['max_angular_rps'], 0.03) if self.verification_mode else self.c['max_angular_rps']
        if self.segment < len(path):
            goal = np.asarray(path[self.segment])
            delta = goal-a[:2, 3]
            if not self.planner.segment_safe(a[:2, 3], goal):
                self.stop('Deviation makes current segment unsafe', 'FAULT')
                return self.command
            error = wrap(math.atan2(delta[1], delta[0])-yaw(a))
            angular = float(np.clip(error, -max_angular, max_angular))
            if abs(error) < 0.10:
                linear = min(max_linear, 0.4*float(np.linalg.norm(delta)))
        else:
            error = wrap(item['pose'][2]-yaw(a))
            if abs(error) <= math.radians(self.c['yaw_tolerance_deg']):
                self.state, self.reason = 'SETTLING', 'Waiting for two seconds of stillness'
                self.command = (0.0, 0.0)
                self.segment_started = now
                return self.command
            angular = float(np.clip(error, -max_angular, max_angular))
        dv = self.c['linear_acceleration_mps2']*dt
        dw = self.c['angular_acceleration_rps2']*dt
        self.command = (float(np.clip(linear, self.command[0]-dv, self.command[0]+dv)),
                        float(np.clip(angular, self.command[1]-dw, self.command[1]+dw)))
        return self.command

    def capture_finished(self, now):
        if self.state != 'CAPTURING':
            raise ValueError('Capture is no longer active')
        self.capture_ready = False
        if self.manual_capture:
            self.stop('Stationary measurement saved')
            return
        self.index += 1
        if self.index >= len(self.plan):
            self.stop('All measurement poses recorded', 'COMPLETED')
        else:
            self.segment = 1
            self.segment_started = now
            self.state, self.reason = 'MOVING', 'Driving to next measurement pose'
