#!/usr/bin/env python3
"""Losslessly archive CLOSED SQLite bag splits, verifying before local removal."""
import copy
import errno
import hashlib
import json
import os
import struct
import subprocess
import time
from pathlib import Path

import rosbag2_py as bag
import yaml

ROOT = Path(__file__).resolve().parent
REMOTE = os.environ.get('UAS_CLOUD_REMOTE', 'uas_gdrive_private:multilink_autonomy_zab_20261002')
SPOOL = Path('/mnt/c/Users/j-sug/AppData/Local/uas-cloud-spool')
PERIODS = {'/dragon/imu/raw': .001, '/dragon/lidar/points': .1,
           **{f'/dragon/cam_link{i}/image': .05 for i in (1, 3, 4)}}


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2) + '\n')
    tmp.replace(path)


def digest(uri, storage):
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(uri), storage_id=storage), bag.ConverterOptions('', ''))
    hashes, counts, sensors = {}, {}, {}
    schemas = {t.name: (t.type, t.serialization_format, t.offered_qos_profiles)
               for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        h = hashes.setdefault(topic, hashlib.sha256())
        h.update(struct.pack('<qQ', stamp, len(data)))
        h.update(data)
        counts[topic] = counts.get(topic, 0) + 1
        if topic in PERIODS:
            sec, ns = struct.unpack('<iI' if data[1] == 1 else '>iI', data[4:12])
            t = sec + ns * 1e-9
            s = sensors.setdefault(topic, dict(count=0, first=t, last=None,
                                               max_gap_s=0., missing_intervals=0, nonmonotonic=0))
            if s['last'] is not None:
                dt = t - s['last']
                s['max_gap_s'] = max(s['max_gap_s'], dt)
                if dt <= 0:
                    s['nonmonotonic'] += 1
                if dt > PERIODS[topic] * 1.5:
                    s['missing_intervals'] += max(1, round(dt / PERIODS[topic]) - 1)
            s['last'] = t
            s['count'] += 1
    return dict(hashes={k: h.hexdigest() for k, h in hashes.items()},
                counts=counts, schemas=schemas, sensors=sensors)


def file_md5(path):
    h = hashlib.md5()
    with path.open('rb', buffering=0) as f:
        offset = 0
        while True:
            for attempt in range(8):
                try:
                    f.seek(offset)
                    data = f.read(256 * 1024)
                    break
                except OSError as exc:
                    if exc.errno != errno.ENOMEM or attempt == 7:
                        raise
                    time.sleep(min(attempt + 1, 5))
            if not data:
                break
            h.update(data)
            offset += len(data)
    return h.hexdigest()


def upload(path, destination, log):
    subprocess.run(['rclone', 'copyto', str(path), destination, '--immutable', '--checksum',
                    '--tpslimit', '2', '--retries', '5', '--low-level-retries', '10',
                    '--stats', '30s', '--stats-one-line', '--log-level', 'INFO'],
                   stdout=log, stderr=log, check=True)
    # Newly created Drive objects can be absent from an immediate lookup.
    for attempt in range(6):
        result = subprocess.run(['rclone', 'lsjson', destination, '--stat', '--hash',
                                 '--tpslimit', '2'], capture_output=True, text=True)
        if result.returncode == 0:
            break
        log.write(result.stderr); log.flush()
        time.sleep(min(2**attempt, 15))
    result.check_returncode()
    remote = json.loads(result.stdout)
    actual = file_md5(path)
    hashes = {k.lower(): v.lower() for k, v in remote.get('Hashes', {}).items()}
    if remote['Size'] != path.stat().st_size or hashes.get('md5') != actual:
        raise RuntimeError('Uploaded file size or MD5 mismatch: ' + destination)
    return actual


def prepare_remote(run):
    """Create parents once, before parallel writers (Drive permits duplicate names)."""
    subprocess.run(['rclone', 'mkdir', f'{REMOTE}/{run.name}/rosbag', '--tpslimit', '2'], check=True)
    result = subprocess.run(['rclone', 'lsjson', REMOTE, '--dirs-only'],
                            capture_output=True, text=True, check=True)
    if sum(item['Name'] == run.name for item in json.loads(result.stdout)) != 1:
        raise RuntimeError('Cloud run directory is missing or duplicated')


def archive_split(run, source):
    receipt = run / 'cloud_chunks' / (source.stem + '.json')
    if receipt.exists():
        return json.loads(receipt.read_text())
    # Never touch a file still owned by the recorder or another process.
    for proc in Path('/proc').glob('[0-9]*/fd'):
        try:
            for fd in proc.iterdir():
                try:
                    if fd.resolve() == source.resolve():
                        raise RuntimeError('Source is still open: ' + str(source))
                except OSError:
                    pass
        except (PermissionError, FileNotFoundError):
            pass
    staging = SPOOL / run.name / source.stem
    staging.mkdir(parents=True, exist_ok=True)
    converted = staging / 'converted'
    options = staging / 'convert.yaml'
    options.write_text(yaml.safe_dump({'output_bags': [dict(uri=str(converted), storage_id='mcap',
        storage_config_uri=str(ROOT / 'mcap_cloud.yaml'), all=True)]}))
    log_path = run / 'cloud_chunks' / (source.stem + '.log')
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open('a') as log:
        if not (converted / 'metadata.yaml').exists():
            if converted.exists():
                raise RuntimeError('Incomplete prior conversion; inspect ' + str(converted))
            subprocess.run(['ros2', 'bag', 'convert', '-i', str(source), '-o', str(options)],
                           stdout=log, stderr=log, check=True)
        files = list(converted.glob('*.mcap'))
        if len(files) != 1:
            raise RuntimeError('Expected exactly one converted MCAP')
        dest = files[0]
        original, compressed = digest(source, 'sqlite3'), digest(dest, 'mcap')
        if original != compressed:
            raise RuntimeError('Conversion changed message data/order/times/schema: ' + str(source))
        remote = f'{REMOTE}/{run.name}/rosbag/{source.stem}.mcap'
        checksum = upload(dest, remote, log)
        result = dict(source=source.name, source_bytes=source.stat().st_size,
                      cloud_file=source.stem + '.mcap', remote=remote,
                      compressed_bytes=dest.stat().st_size, md5=checksum,
                      verification=compressed,
                      file_metadata=yaml.safe_load((converted/'metadata.yaml').read_text())['rosbag2_bagfile_information'])
        save(receipt, result)
        # Only after exact-message comparison AND confirmed remote checksum.
        original_target = source.resolve() if source.is_symlink() else None
        source.unlink()
        if original_target is not None:
            if original_target.parent != run/'rosbag_before_cloud':
                raise RuntimeError('Unexpected source link target; preserving ' + str(original_target))
            original_target.unlink()
        dest.unlink()
        return result


def finalize(run):
    original = yaml.safe_load((run / 'rosbag/metadata.yaml').read_text())
    meta = copy.deepcopy(original)
    info = meta['rosbag2_bagfile_information']
    receipts = [json.loads((run/'cloud_chunks'/(Path(p).stem+'.json')).read_text())
                for p in info['relative_file_paths']]
    info['storage_identifier'] = 'mcap'
    info['relative_file_paths'] = [r['cloud_file'] for r in receipts]
    for f, r in zip(info.get('files', []), receipts):
        f['path'] = r['cloud_file']
    info['compression_format'] = ''
    info['compression_mode'] = ''
    counts = {}
    sensors = {}
    for r in receipts:
        for name, count in r['verification']['counts'].items():
            counts[name] = counts.get(name, 0) + count
        for name, s in r['verification']['sensors'].items():
            if name not in sensors:
                sensors[name] = dict(s)
                continue
            acc = sensors[name]
            dt = s['first'] - acc['last']
            acc['max_gap_s'] = max(acc['max_gap_s'], s['max_gap_s'], dt)
            acc['nonmonotonic'] += s['nonmonotonic'] + int(dt <= 0)
            acc['missing_intervals'] += s['missing_intervals']
            if dt > PERIODS[name]*1.5:
                acc['missing_intervals'] += max(1, round(dt/PERIODS[name])-1)
            acc['count'] += s['count']
            acc['last'] = s['last']
    if sum(counts.values()) != info['message_count']:
        raise RuntimeError('Archived total message count mismatch')
    for t in info['topics_with_message_count']:
        if counts.get(t['topic_metadata']['name'], 0) != t['message_count']:
            raise RuntimeError('Archived topic count mismatch')
    if set(sensors) != set(PERIODS):
        raise RuntimeError('Missing raw sensor topics')
    result = dict(duration_s=info['duration']['nanoseconds']/1e9, sensors=sensors,
                  total_messages=info['message_count'], bytes=sum(r['compressed_bytes'] for r in receipts),
                  source_bytes=sum(r['source_bytes'] for r in receipts),
                  remote=f'{REMOTE}/{run.name}/rosbag', format='mcap', lossless_verified=True)
    save(run/'raw_bag_validation.json', result)
    cloud_meta = run/'cloud_metadata.yaml'
    cloud_meta.write_text(yaml.safe_dump(meta, sort_keys=False))
    with (run/'cloud_chunks/finalize.log').open('a') as log:
        upload(cloud_meta, result['remote']+'/metadata.yaml', log)
        upload(run/'raw_bag_validation.json', f'{REMOTE}/{run.name}/raw_bag_validation.json', log)
        upload(run/'route_status.json', f'{REMOTE}/{run.name}/route_status.json', log)
    return result
