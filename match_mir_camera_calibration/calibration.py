"""Joint extrinsic fit on stored corners; executable without ROS or hardware."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
import yaml

from .config import marker_names, validate
from .dataset import load_dataset, write_json
from .geometry import distance, inverse, marker_points, pack, transform, unpack, values
from .vision import Detector, camera_model, hypotheses


SIDES = ('left', 'right')


class CalibrationError(ValueError):
    pass


def redetect_measurements(root, measurements, config):
    """Replace detections in memory only; lossless images and measurements stay intact."""
    root = Path(root).resolve()
    detector = Detector(config)
    output = []
    report = dict(detector=detector.description(), images=0, recorded_detections=0,
                  redetected_detections=0, added_id_observations=0, lost_id_observations=0,
                  methods={})
    for measurement in measurements:
        updated = dict(measurement, images=[])
        for record in measurement['images']:
            path = (root/record['path']).resolve()
            if not path.is_relative_to(root):
                raise CalibrationError('Image path escapes the measurement folder')
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise CalibrationError(f'Cannot read original image: {record["path"]}')
            detections = detector.detect(image, record['camera_info'])
            updated['images'].append(dict(record, detections=detections, detector=detector.description()))
            old = {d['id'] for d in record['detections']}
            new = {d['id'] for d in detections}
            report['images'] += 1
            report['recorded_detections'] += len(record['detections'])
            report['redetected_detections'] += len(detections)
            report['added_id_observations'] += len(new-old)
            report['lost_id_observations'] += len(old-new)
            for detection in detections:
                method = detection['detection_method']
                report['methods'][method] = report['methods'].get(method, 0)+1
        output.append(updated)
    return output, report


def observations(measurements, config):
    result = []
    frames = {}
    groups = []
    for measurement in measurements:
        if not measurement['images']:
            continue
        first = measurement['images'][0]
        reference = inverse(transform(first['pose_b'])) @ transform(first['pose_a'])
        group_id = None
        for group_pose, group_name in groups:
            delta, angle = distance(group_pose, reference)
            if delta < 0.03 and angle < np.radians(3.0):
                group_id = group_name
                break
        if group_id is None:
            group_id = measurement['id']
            groups.append((reference, group_id))
        for image in measurement['images']:
            side = image['camera']
            if side not in SIDES:
                raise CalibrationError('Unknown camera in dataset')
            frame = image['frame_id']
            if side in frames and frames[side] != frame:
                raise CalibrationError('Camera optical frame changed during recording')
            frames[side] = frame
            k, d = camera_model(image['camera_info'])
            relative = inverse(transform(image['pose_b'])) @ transform(image['pose_a'])
            for detection in image['detections']:
                name = detection['marker']
                if name not in config['markers'] or detection['id'] != config['markers'][name]['id']:
                    raise CalibrationError('Marker configuration differs from recorded detections')
                corners = np.asarray(detection['corners'], float)
                if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
                    raise CalibrationError('Invalid image corners')
                result.append({'side': side, 'marker': name, 'corners': corners,
                               'k': k, 'd': d, 'relative': relative,
                               'waypoint': group_id,
                               'candidates': detection.get('pnp_candidates') or hypotheses(
                                   corners, config['markers'][name]['length_m'], k, d)})
    if len(set(frames.values())) != 2:
        raise CalibrationError('Two distinct, stable camera optical frames are required')
    return result, frames


def check_coverage(obs, minimum=6, markers=None):
    if markers is None:
        observed = {o['marker'] for o in obs}
        prefix = 'front' if any(name.startswith('front_') for name in observed) else 'rear'
        markers = (prefix+'_left', prefix+'_right')
    nodes = set(SIDES + tuple(markers))
    edges = {node: set() for node in nodes}
    coverage = {node: set() for node in nodes}
    for o in obs:
        if o['side'] not in SIDES or o['marker'] not in markers:
            raise CalibrationError('Observation does not belong to the configured camera/marker pair')
        edges[o['side']].add(o['marker'])
        edges[o['marker']].add(o['side'])
        coverage[o['side']].add(o['waypoint'])
        coverage[o['marker']].add(o['waypoint'])
    seen, pending = set(), ['left']
    while pending:
        node = pending.pop()
        if node not in seen:
            seen.add(node)
            pending.extend(edges[node]-seen)
    if seen != nodes:
        raise CalibrationError('Disconnected camera/marker observations: at least one marker must '
                               'be observed by both cameras across the session')
    if any(len(v) < minimum for v in coverage.values()):
        raise CalibrationError(f'Each camera and marker needs observations at at least {minimum} distinct measurement poses')
    return {key: len(value) for key, value in coverage.items()}


class JointProblem:
    def __init__(self, config, obs):
        self.config, self.obs = config, obs
        self.markers = marker_names(config)
        self.points = {name: marker_points(config['markers'][name]['length_m']) for name in self.markers}
        self.anchor_index = 12 + 6*self.markers.index(config['height_anchor']['marker']) + 5
        self.free = np.delete(np.arange(24), self.anchor_index)

    def expand(self, x):
        full = np.zeros(24)
        full[self.free] = x
        full[self.anchor_index] = self.config['height_anchor']['z_m']
        return full

    def transforms(self, x):
        full = self.expand(x)
        return {name: unpack(full[6*i:6*i+6]) for i, name in enumerate(SIDES+self.markers)}

    def residual(self, x, obs=None):
        ts = self.transforms(x)
        out = []
        for o in self.obs if obs is None else obs:
            t = inverse(ts[o['side']]) @ o['relative'] @ ts[o['marker']]
            points = self.points[o['marker']]
            z = (points @ t[:3, :3].T + t[:3, 3])[:, 2]
            predicted = cv2.projectPoints(points, cv2.Rodrigues(t[:3, :3])[0], t[:3, 3], o['k'], o['d'])[0].reshape(4, 2)
            out.extend((predicted-o['corners']).ravel())
            # A zero-cost cheirality term forbids behind-camera fits.
            out.extend(10000*np.minimum(z-0.005, 0))
        return np.asarray(out)

    def errors(self, x, obs):
        ts = self.transforms(x)
        pixel, translation, rotation = [], [], []
        for o in obs:
            predicted_t = inverse(ts[o['side']]) @ o['relative'] @ ts[o['marker']]
            xyz = self.points[o['marker']] @ predicted_t[:3, :3].T + predicted_t[:3, 3]
            if np.any(xyz[:, 2] <= 0.005):
                raise CalibrationError('Fit places a marker behind a camera')
            predicted = cv2.projectPoints(self.points[o['marker']], cv2.Rodrigues(predicted_t[:3, :3])[0],
                                          predicted_t[:3, 3], o['k'], o['d'])[0].reshape(4, 2)
            pixel.extend(np.linalg.norm(predicted-o['corners'], axis=1).tolist())
            candidates = o['candidates']
            if candidates:
                # Use the lowest image reprojection hypothesis independently of mocap.
                measured = transform(min(candidates, key=lambda c: c['reprojection_px'])['transform'])
                relative_estimate = ts[o['side']] @ measured @ inverse(ts[o['marker']])
                delta, angle = distance(relative_estimate, o['relative'])
                translation.append(delta)
                rotation.append(float(np.degrees(angle)))
        def stats(items):
            if not items:
                return None
            return {'rms': float(np.sqrt(np.mean(np.square(items)))),
                    'median': float(np.median(items)), 'p95': float(np.percentile(items, 95))}
        return {'corner_error_px': stats(pixel), 'relative_pose_translation_m': stats(translation),
                'relative_pose_rotation_deg': stats(rotation), 'detections': len(obs)}

    def seed(self, guesses, alternative=False):
        ts = {}
        for side in SIDES:
            if side not in guesses:
                raise CalibrationError(f'No initial camera guess for {side}. Record live base_link -> optical TF '
                                       'or provide --initial-guesses YAML with left/right [xyz,qxyzw].')
            ts[side] = transform(guesses[side])
        for marker in self.markers:
            usable = [o for o in self.obs if o['marker'] == marker and o['candidates']]
            if not usable:
                raise CalibrationError(f'No valid positive-depth PnP initialization for {marker}')
            o = usable[0]
            candidate = o['candidates'][min(int(alternative), len(o['candidates'])-1)]
            ts[marker] = inverse(o['relative']) @ ts[o['side']] @ transform(candidate['transform'])
        return np.concatenate([pack(ts[name]) for name in SIDES+self.markers])[self.free]


def fit(config, obs, guesses, starts=4):
    check_coverage(obs, markers=marker_names(config))
    problem = JointProblem(config, obs)
    rng = np.random.default_rng(620)
    best = None
    for index in range(starts):
        seed = problem.seed(guesses, alternative=bool(index % 2))
        if index > 1:
            perturb = np.tile(np.r_[np.full(3, 0.10), np.full(3, 0.03)], 4)[problem.free]
            seed += rng.normal(size=23)*perturb
        result = least_squares(problem.residual, seed, loss='huber', f_scale=2.0,
                               x_scale='jac', max_nfev=700, ftol=1e-9, xtol=1e-9, gtol=1e-8)
        if result.success and (best is None or result.cost < best.cost):
            best = result
    if best is None:
        raise CalibrationError('Joint optimization did not converge')
    # Inspect the physical Jacobian, not Huber's reweighted Jacobian.
    columns = []
    for i in range(len(best.x)):
        step = 1e-6*max(1., abs(best.x[i]))
        plus, minus = best.x.copy(), best.x.copy()
        plus[i] += step
        minus[i] -= step
        columns.append((problem.residual(plus)-problem.residual(minus))/(2*step))
    jac = np.column_stack(columns)
    singular = np.linalg.svd(jac, compute_uv=False)
    rank = int(np.sum(singular > singular[0]*1e-7))
    if rank != 23:
        raise CalibrationError(f'Insufficient pose excitation: Jacobian rank {rank}/23; '
                               'add spatially separated poses and distinct yaw angles')
    report = {'rank': rank, 'unknowns': 23, 'singular_values': singular.tolist(),
              'condition_number': float(singular[0]/singular[-1]),
              'converged': True, 'robust_cost': float(best.cost),
              'training': problem.errors(best.x, obs)}
    return problem, best.x, report


def solve_dataset(path, initial_guesses=None, starts=4, redetect=False):
    root, manifest, raw, measurements = load_dataset(path)
    config = validate(raw)
    redetection = None
    if redetect:
        measurements, redetection = redetect_measurements(root, measurements, config)
        write_json(root/'redetection_report.json', redetection)
    obs, frames = observations(measurements, config)
    coverage = check_coverage(obs, markers=marker_names(config))
    ids = sorted({o['waypoint'] for o in obs})
    if len(ids) < 10:
        raise CalibrationError('At least 10 accepted measurement poses are required for independent validation')
    guesses = dict(config.get('camera_initial_guesses', {}))
    guesses.update(manifest.get('initial_guesses', {}))
    if initial_guesses:
        guesses.update(initial_guesses)
    # Keep ALL images of each fifth measurement out of the validation fit.
    held = set(ids[4::5])
    train = [o for o in obs if o['waypoint'] not in held]
    test = [o for o in obs if o['waypoint'] in held]
    validation_problem, validation_x, validation_fit = fit(config, train, guesses, starts)
    validation = validation_problem.errors(validation_x, test)
    problem, x, report = fit(config, obs, guesses, starts)
    ts = problem.transforms(x)
    report.update(coverage=coverage, accepted_measurements=len(measurements),
                  distinct_poses=len(ids),
                  validation=validation, validation_pose_ids=sorted(held),
                  validation_training_rank=validation_fit['rank'],
                  notes=['Absolute height is conditioned on the supplied marker height.',
                         'PnP pose validation uses the image-best hypothesis; planar ambiguity may increase its error.',
                         'Residuals alone do not certify absolute accuracy of mocap or camera intrinsics.'])
    if redetection is not None:
        report['redetection'] = redetection
    # A converged solution with poor held-out reprojection remains an explicit draft.
    quality = ('validated' if validation['corner_error_px']['p95'] <= 3.0 and
               report['condition_number'] <= 1e6 else 'draft')
    export = {'schema_version': 1, 'quality': quality, 'units': 'metres',
              'height_anchor': config['height_anchor'], 'transforms': {}, 'report': report}
    for name in SIDES+problem.markers:
        parent = f"{config['observer_robot'] if name in SIDES else config['target_robot']}/base_link"
        child = frames[name] if name in SIDES else f"{config['target_robot']}/aruco_{name}"
        v = values(ts[name])
        export['transforms'][name] = {'parent_frame': parent, 'child_frame': child,
                                      'translation': v[:3], 'quaternion_xyzw': v[3:]}
    output = root/'calibration.yaml'
    temporary = output.with_suffix('.yaml.tmp')
    temporary.write_text(yaml.safe_dump(export, sort_keys=False))
    temporary.replace(output)
    write_json(root/'quality_report.json', dict(report, quality=quality))
    return export


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_dir')
    parser.add_argument('--initial-guesses', help='YAML: left/right [x,y,z,qx,qy,qz,qw], or session config')
    parser.add_argument('--starts', type=int, default=4)
    parser.add_argument('--redetect', action='store_true',
                        help='Re-detect original lossless images with the current detector; preserve stored measurements')
    args = parser.parse_args(argv)
    guesses = None
    try:
        if not 1 <= args.starts <= 12:
            raise CalibrationError('--starts must be between 1 and 12')
        if args.initial_guesses:
            guesses = yaml.safe_load(Path(args.initial_guesses).expanduser().read_text())
            guesses = guesses.get('camera_initial_guesses', guesses)
        result = solve_dataset(args.session_dir, guesses, args.starts, redetect=args.redetect)
        print(json.dumps({'quality': result['quality'], 'validation': result['report']['validation'],
                          'output': str(Path(args.session_dir).expanduser()/'calibration.yaml')}, indent=2))
    except (ValueError, TypeError, OSError, KeyError, np.linalg.LinAlgError, cv2.error) as exc:
        try:
            root = Path(args.session_dir).expanduser()
            if root.is_dir():
                write_json(root/'quality_report.json', {'quality': 'underdetermined_or_invalid', 'reason': str(exc)})
                # Do not leave a stale successful calibration beside a failed re-fit.
                previous = root/'calibration.yaml'
                if previous.exists():
                    previous.replace(root/'calibration.previous.yaml')
        except OSError:
            pass
        parser.exit(1, f'Calibration rejected: {exc}\n')


if __name__ == '__main__':
    main()
