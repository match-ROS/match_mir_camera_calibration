"""ArUco corners and both planar PnP hypotheses, without ROS dependencies."""
import cv2
import numpy as np

from .geometry import marker_points, values


def camera_model(info):
    if info['distortion_model'] not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('Only pinhole plumb_bob/rational_polynomial CameraInfo is supported')
    k = np.asarray(info['k'], float).reshape(3, 3)
    d = np.asarray(info['d'], float)
    if not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('CameraInfo is uncalibrated or nonfinite')
    if len(d) not in (0, 4, 5, 8, 12, 14):
        raise ValueError('Unsupported distortion coefficient count')
    if info.get('binning_x', 0) > 1 or info.get('binning_y', 0) > 1:
        raise ValueError('Binned images require a separately adjusted CameraInfo')
    roi = info.get('roi', {})
    if roi.get('x_offset', 0) or roi.get('y_offset', 0):
        raise ValueError('Cropped images are not supported')
    if roi.get('width', 0) not in (0, info['width']) or roi.get('height', 0) not in (0, info['height']):
        raise ValueError('CameraInfo ROI must describe the full image')
    return k, d


def hypotheses(corners, length, k, d):
    points = marker_points(length)
    result = cv2.solvePnPGeneric(points, np.asarray(corners, float), k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    poses = []
    for rvec, tvec in zip(result[1], result[2]):
        if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
            continue
        rotation = cv2.Rodrigues(rvec)[0]
        xyz = points @ rotation.T + tvec.reshape(3)
        if np.any(xyz[:, 2] <= 0):
            continue
        projected = cv2.projectPoints(points, rvec, tvec, k, d)[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=1))))
        if not np.isfinite(error):
            continue
        t = np.eye(4)
        t[:3, :3], t[:3, 3] = rotation, tvec.reshape(3)
        poses.append({'transform': values(t), 'reprojection_px': error})
    return sorted(poses, key=lambda p: p['reprojection_px'])


class Detector:
    def __init__(self, config):
        self.markers = config['markers']
        self.names = {item['id']: name for name, item in self.markers.items()}
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
        self.parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                           else cv2.aruco.DetectorParameters_create())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def detect(self, image, info):
        k, d = camera_model(info)
        if image.shape[:2] != (info['height'], info['width']):
            raise ValueError('Image dimensions do not match CameraInfo')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        detections = []
        if ids is not None:
            if len(set(ids.flatten().tolist())) != len(ids):
                raise ValueError('Duplicate marker ID in image; use unique physical markers')
            for corner, mid in zip(corners, ids.flatten()):
                if int(mid) not in self.names:
                    continue
                name = self.names[int(mid)]
                c = corner.reshape(4, 2)
                poses = hypotheses(c, self.markers[name]['length_m'], k, d)
                detections.append({'marker': name, 'id': int(mid), 'corners': c.tolist(),
                                   'pnp_candidates': poses})
        return detections

    @staticmethod
    def overlay(image, detections, info=None, rotate_ccw=False):
        result = image.copy()
        for item in detections:
            corners = np.asarray(item['corners'], np.int32)
            cv2.polylines(result, [corners], True, (0, 255, 0), 2)
            if info and item['pnp_candidates']:
                from .geometry import transform
                pose = transform(item['pnp_candidates'][0]['transform'])
                k, d = camera_model(info)
                cv2.drawFrameAxes(result, k, d, cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3], 0.04, 2)
        if rotate_ccw:
            result = cv2.rotate(result, cv2.ROTATE_90_COUNTERCLOCKWISE)
        for item in detections:
            corners = np.asarray(item['corners'], float)
            if rotate_ccw:
                corners = np.column_stack((corners[:, 1], image.shape[1]-1-corners[:, 0]))
            x = int(np.clip(corners[:, 0].min(), 0, max(0, result.shape[1]-225)))
            y = max(16, int(corners[:, 1].min())-25)
            label = 'hinten links' if item['marker'] == 'rear_left' else 'hinten rechts'
            lines = [f"ID {item['id']} {label}"]
            if item['pnp_candidates']:
                xyz = item['pnp_candidates'][0]['transform'][:3]
                lines.append('XYZ ' + '/'.join(f'{v:.2f}' for v in xyz) + ' m')
            for offset, line in enumerate(lines):
                point = (x, min(result.shape[0]-4, y+offset*16))
                cv2.putText(result, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3)
                cv2.putText(result, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
        return result
