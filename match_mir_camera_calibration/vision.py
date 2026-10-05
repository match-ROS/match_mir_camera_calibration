"""ArUco corners and both planar PnP hypotheses, without ROS dependencies."""
import cv2
import numpy as np

from .geometry import marker_points, values
from .config import MARKER_LABELS


def dictionary_bits(dictionary, mid):
    get_bits = (cv2.aruco.Dictionary.getBitsFromByteList if hasattr(cv2.aruco, 'ArucoDetector')
                else cv2.aruco.Dictionary_getBitsFromByteList)
    return get_bits(dictionary.bytesList[mid:mid+1], dictionary.markerSize)


def known_dictionary(dictionary, ids):
    """Keep original codes/orientations; bound correction against ALL family IDs.

    OpenCV's AprilTag dictionaries have maxCorrectionBits=0 in some releases.
    Derive a conservative radius instead of relaxing decoding arbitrarily.
    Including rotated copies also protects the correspondence of the four corners.
    """
    codes = np.asarray([dictionary_bits(dictionary, i)
                        for i in range(len(dictionary.bytesList))])
    minimum = dictionary.markerSize**2
    for mid in ids:
        for rotation in range(4):
            differences = np.count_nonzero(codes[mid] != np.rot90(codes, rotation, axes=(1, 2)), axis=(1, 2))
            if rotation == 0:
                differences[mid] = dictionary.markerSize**2
            minimum = min(minimum, int(differences.min()))
    correction = min(2, max(0, (minimum-1)//2))
    selected = np.array(dictionary.bytesList[ids], copy=True)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        subset = cv2.aruco.Dictionary(selected, dictionary.markerSize, correction)
    else:
        subset = cv2.aruco.Dictionary_create(len(ids), dictionary.markerSize)
        subset.bytesList = selected
        subset.maxCorrectionBits = correction
    return subset, minimum


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
        self.robust = config.get('robust_detection', True)
        self.ids = list(self.names)
        self.recovery_dictionary, self.code_distance = known_dictionary(self.dictionary, self.ids)
        self.recovery_parameters = (cv2.aruco.DetectorParameters() if self.detector
                                    else cv2.aruco.DetectorParameters_create())
        self.recovery_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.recovery_parameters.cornerRefinementWinSize = 3
        self.recovery_parameters.adaptiveThreshWinSizeMax = 53
        self.recovery_parameters.adaptiveThreshWinSizeStep = 4
        self.recovery_parameters.errorCorrectionRate = 1.0
        self.recovery_parameters.maxErroneousBitsInBorderRate = 0.15
        self.recovery_detector = (cv2.aruco.ArucoDetector(self.recovery_dictionary, self.recovery_parameters)
                                 if self.detector else None)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.expected_bits = {mid: dictionary_bits(self.dictionary, mid) for mid in self.ids}

    def description(self):
        return {'version': 2, 'opencv': cv2.__version__, 'robust_detection': self.robust,
                'known_ids': self.ids, 'minimum_code_distance': self.code_distance,
                'recovery_correction_limit_bits': self.recovery_dictionary.maxCorrectionBits,
                'recovery_methods': ['known_ids', 'upscaled', 'contrast'],
                'recovery_max_pnp_error_px': 1.5,
                'corners': 'original image coordinates, observed in this frame; no temporal/mocap predictions'}

    def _candidates(self, gray, recovery=False):
        detector = self.recovery_detector if recovery else self.detector
        if detector:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.recovery_dictionary if recovery else self.dictionary,
                parameters=self.recovery_parameters if recovery else self.parameters)
        result = []
        if ids is not None:
            for c, mid in zip(corners, ids.flatten()):
                mid = self.ids[int(mid)] if recovery else int(mid)
                if mid in self.names:
                    result.append((mid, c.reshape(4, 2)))
        if len({mid for mid, _ in result}) != len(result):
            raise ValueError('Duplicate marker ID in image; use unique physical markers')
        return result

    def _refine_recovered(self, gray, corners):
        height, width = gray.shape
        if (not np.all(np.isfinite(corners)) or np.any(corners < 3) or
                np.any(corners[:, 0] > width-4) or np.any(corners[:, 1] > height-4) or
                not cv2.isContourConvex(corners.astype(np.float32))):
            return None
        side = np.linalg.norm(corners-np.roll(corners, 1, axis=0), axis=1).min()
        if side < 1.5*(self.dictionary.markerSize+2):
            return None
        # Keep the subpixel window away from neighbouring code cells. Refine on
        # the original pixels even when the candidate came from resize/CLAHE.
        window = max(1, min(3, int(side/(2*(self.dictionary.markerSize+2)))))
        refined = np.asarray(corners, np.float32).reshape(4, 1, 2).copy()
        cv2.cornerSubPix(gray, refined, (window, window), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .02))
        refined = refined.reshape(4, 2)
        if np.max(np.linalg.norm(refined-corners, axis=1)) > 2.0:
            return None
        return refined

    def _verify_recovered(self, gray, corners, mid):
        """Recheck code and black border on ORIGINAL pixels after corner refinement."""
        cells = self.dictionary.markerSize+2
        size = cells*8
        target = np.array([[0, 0], [size-1, 0], [size-1, size-1], [0, size-1]], np.float32)
        patch = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(corners.astype(np.float32), target),
                                    (size, size))
        if patch.std() < 5:
            return None
        binary = cv2.threshold(patch, 0, 1, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        # Ignore cell edges, which mix black and white at this resolution.
        bits = np.array([[binary[8*y+2:8*y+6, 8*x+2:8*x+6].mean() > .5
                          for x in range(cells)] for y in range(cells)])
        border_errors = int(bits.sum()-bits[1:-1, 1:-1].sum())
        if border_errors > self.recovery_parameters.maxErroneousBitsInBorderRate*self.dictionary.markerSize**2:
            return None
        errors = int(np.count_nonzero(bits[1:-1, 1:-1] != self.expected_bits[mid]))
        return errors if errors <= self.recovery_dictionary.maxCorrectionBits else None

    def detect(self, image, info):
        k, d = camera_model(info)
        if image.shape[:2] != (info['height'], info['width']):
            raise ValueError('Image dimensions do not match CameraInfo')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        detections = {}
        for mid, corners in self._candidates(gray):
            name = self.names[mid]
            detections[mid] = {'marker': name, 'id': mid, 'corners': corners.tolist(),
                               'pnp_candidates': hypotheses(corners, self.markers[name]['length_m'], k, d),
                               'detection_method': 'standard', 'correction_limit_bits': int(
                                   self.dictionary.maxCorrectionBits*self.parameters.errorCorrectionRate)}
        if self.robust and len(detections) < len(self.ids):
            for method in ('known_ids', 'upscaled', 'contrast'):
                if method == 'upscaled':
                    source = cv2.resize(gray, None, fx=2., fy=2., interpolation=cv2.INTER_LINEAR)
                elif method == 'contrast':
                    source = self.clahe.apply(gray)
                else:
                    source = gray
                for mid, corners in self._candidates(source, recovery=True):
                    if mid in detections:
                        continue  # Preserve the ordinary detector's observed corners.
                    if method == 'upscaled':
                        corners = (corners+.5)/2.-.5  # OpenCV resize maps pixel centres.
                    corners = self._refine_recovered(gray, corners)
                    if corners is None:
                        continue
                    errors = self._verify_recovered(gray, corners, mid)
                    if errors is None:
                        continue
                    # Two IDs may not describe the same physical quadrilateral.
                    if any(np.linalg.norm(corners.mean(axis=0)-np.asarray(other['corners']).mean(axis=0)) <
                           np.linalg.norm(corners[1]-corners[0])*.5 for other in detections.values()):
                        continue
                    name = self.names[mid]
                    poses = hypotheses(corners, self.markers[name]['length_m'], k, d)
                    if not poses or poses[0]['reprojection_px'] > 1.5:
                        continue
                    detections[mid] = {'marker': name, 'id': mid, 'corners': corners.tolist(),
                                       'pnp_candidates': poses, 'detection_method': method,
                                       'correction_limit_bits': self.recovery_dictionary.maxCorrectionBits,
                                       'corrected_bits': errors}
                if len(detections) == len(self.ids):
                    break
        return list(detections.values())

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
            label = MARKER_LABELS.get(item['marker'], item['marker'])
            lines = [f"ID {item['id']} {label}"]
            if item['pnp_candidates']:
                xyz = item['pnp_candidates'][0]['transform'][:3]
                lines.append('XYZ ' + '/'.join(f'{v:.2f}' for v in xyz) + ' m')
            for offset, line in enumerate(lines):
                point = (x, min(result.shape[0]-4, y+offset*16))
                cv2.putText(result, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3)
                cv2.putText(result, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
        return result
