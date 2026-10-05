"""Lossless, versioned, inspectable session files; rejected bursts never fit."""
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

import cv2
import yaml


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


class Dataset:
    def __init__(self, config, initial_guesses):
        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
        self.path = Path(config['output_dir']).expanduser() / name
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/'measurements').mkdir()
        (self.path/'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        write_json(self.path/'manifest.json', {
            'schema_version': 1, 'world_frame': 'mocap',
            'pose_convention': 'parent_T_child; xyz metres; quaternion xyzw',
            'initial_guesses': initial_guesses,
            'timestamp_semantics': 'MiR exposure/source headers; mocap host publication headers; '
                                   'poses paired at receipt only in guarded stationary bursts',
        })
        self.pose_log = (self.path/'poses.jsonl').open('a', buffering=1)
        self.event_log = (self.path/'events.jsonl').open('a', buffering=1)
        self.count = 0
        self.accepted_count = 0
        self.pending = None

    def pose(self, record):
        self.pose_log.write(json.dumps(record, allow_nan=False)+'\n')

    def event(self, record):
        self.event_log.write(json.dumps(record, allow_nan=False)+'\n')

    def begin(self, waypoint_id, started_at):
        if self.pending is not None:
            raise ValueError('A measurement is already open')
        folder = self.path/'measurements'/f'{self.count:06d}'
        folder.mkdir()
        self.pending = {'id': f'{self.count:06d}', 'waypoint_id': waypoint_id,
                        'started_at': started_at, 'accepted': False, 'images': []}
        self.count += 1
        return folder

    def image(self, image, record):
        if self.pending is None:
            raise ValueError('No active measurement')
        filename = f"{record['camera']}_{len(self.pending['images']):03d}.png"
        relative = Path('measurements')/self.pending['id']/filename
        if not cv2.imwrite(str(self.path/relative), image):
            raise OSError('Could not write lossless image')
        record = dict(record, path=str(relative))
        self.pending['images'].append(record)

    def finish(self, accepted, reason, finished_at):
        if self.pending is None:
            return
        self.pending.update(accepted=bool(accepted), reason=reason, finished_at=finished_at)
        write_json(self.path/'measurements'/self.pending['id']/'measurement.json', self.pending)
        if accepted:
            self.accepted_count += 1
        self.pending = None

    def close(self):
        self.pose_log.close()
        self.event_log.close()


def load_dataset(path):
    root = Path(path).expanduser()
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema_version') != 1 or manifest.get('world_frame') != 'mocap':
        raise ValueError('Unsupported dataset schema or world frame')
    config = yaml.safe_load((root/'config.yaml').read_text())
    measurements = [json.loads(file.read_text()) for file in sorted((root/'measurements').glob('*/measurement.json'))]
    return root, manifest, config, [m for m in measurements if m.get('accepted')]
