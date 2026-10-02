"""Conservative circle-envelope planner in the raw mocap XY plane."""
import heapq
import math

import numpy as np

from .geometry import wrap


class PlanningError(ValueError):
    pass


class Planner:
    def __init__(self, config, observer_xy):
        self.c = config
        self.observer = np.asarray(observer_xy, dtype=float)
        v, a = config['max_linear_mps'], config['linear_acceleration_mps2']
        # Includes loss-of-GUI detection, mocap age and a physical reaction bound.
        latency = max(config['heartbeat_timeout_sec'], config['mocap_timeout_sec'])
        self.reserve = config['margin_m'] + v * (latency + config['reaction_time_sec']) + v*v/(2*a)
        pad = config['target_radius_m'] + self.reserve
        b = config['bounds']
        self.lower = np.array([b['x_min'] + pad, b['y_min'] + pad])
        self.upper = np.array([b['x_max'] - pad, b['y_max'] - pad])
        self.exclusion = config['target_radius_m'] + config['observer_radius_m'] + self.reserve
        if np.any(self.lower >= self.upper):
            raise PlanningError('Rectangle too small for robot envelope and stopping reserve')

    def safe(self, xy):
        p = np.asarray(xy, dtype=float)
        return bool(p.shape == (2,) and np.all(np.isfinite(p)) and
                    np.all(p >= self.lower - 1e-9) and np.all(p <= self.upper + 1e-9) and
                    np.linalg.norm(p - self.observer) >= self.exclusion)

    def segment_safe(self, a, b):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        if not self.safe(a) or not self.safe(b):
            return False
        delta = b - a
        f = np.clip(np.dot(self.observer - a, delta) / max(np.dot(delta, delta), 1e-16), 0, 1)
        return np.linalg.norm(a + f * delta - self.observer) >= self.exclusion

    def route(self, start, goal):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if not self.safe(start) or not self.safe(goal):
            raise PlanningError('Start or waypoint violates the inflated safe area')
        if self.segment_safe(start, goal):
            return [start.tolist(), goal.tolist()]
        step = self.c['planner_resolution_m']
        shape = np.floor((self.upper - self.lower) / step).astype(int) + 1
        if np.prod(shape) > 250000:
            raise PlanningError('Planner grid too large; reduce bounds or increase resolution')
        def point(index):
            return self.lower + np.asarray(index) * step
        def nearby(p):
            center = np.rint((p - self.lower) / step).astype(int)
            options = []
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    idx = tuple(center + [dx, dy])
                    if all(0 <= idx[k] < shape[k] for k in (0, 1)) and self.segment_safe(p, point(idx)):
                        options.append(idx)
            return options
        starts, goals = nearby(start), set(nearby(goal))
        if not starts or not goals:
            raise PlanningError('No safe connection to the planning grid')
        queue, costs, parents = [], {}, {}
        for idx in starts:
            cost = float(np.linalg.norm(point(idx) - start))
            costs[idx] = cost
            parents[idx] = None
            heapq.heappush(queue, (cost + np.linalg.norm(point(idx) - goal), cost, idx))
        end = None
        while queue:
            _, cost, idx = heapq.heappop(queue)
            if cost > costs[idx] + 1e-10:
                continue
            if idx in goals:
                end = idx
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (idx[0]+dx, idx[1]+dy)
                if not all(0 <= nxt[k] < shape[k] for k in (0, 1)) or not self.segment_safe(point(idx), point(nxt)):
                    continue
                nc = cost + step * math.hypot(dx, dy)
                if nc < costs.get(nxt, float('inf')):
                    costs[nxt], parents[nxt] = nc, idx
                    heapq.heappush(queue, (nc + np.linalg.norm(point(nxt)-goal), nc, nxt))
        if end is None:
            raise PlanningError('No safe route around the observer robot')
        path = [goal.tolist()]
        while end is not None:
            path.append(point(end).tolist())
            end = parents[end]
        path.append(start.tolist())
        path.reverse()
        # Remove redundant grid points without cutting across the exclusion circle.
        simplified = [path[0]]
        index = 0
        while index < len(path)-1:
            next_index = len(path)-1
            while not self.segment_safe(path[index], path[next_index]):
                next_index -= 1
            simplified.append(path[next_index])
            index = next_index
        return simplified

    def raster(self):
        grid = self.c['grid']
        def axis(lo, hi, n):
            return np.linspace(lo + 0.01, hi - 0.01, n) if n > 1 else [(lo+hi)/2]
        result = []
        for row, y in enumerate(axis(self.lower[1], self.upper[1], grid['ny'])):
            xs = list(axis(self.lower[0], self.upper[0], grid['nx']))
            if row % 2:
                xs.reverse()
            for x in xs:
                if not self.safe([x, y]):
                    continue
                rear_to_observer = math.atan2(y-self.observer[1], x-self.observer[0])
                for offset in grid['yaw_offsets_deg']:
                    result.append([float(x), float(y), wrap(rear_to_observer + math.radians(offset))])
        if not result:
            raise PlanningError('No valid raster poses remain')
        return result

    def plan(self, start, waypoints=None):
        waypoints = self.raster() if waypoints is None else waypoints
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 1296:
            raise PlanningError('A nonempty list of at most 1296 waypoints is required')
        result, previous = [], list(start[:2])
        for pose in waypoints:
            if len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
                raise PlanningError('Each waypoint must be finite [x,y,yaw radians]')
            goal = [float(pose[0]), float(pose[1]), wrap(pose[2])]
            result.append({'pose': goal, 'path': self.route(previous, goal[:2])})
            previous = goal[:2]
        return result
