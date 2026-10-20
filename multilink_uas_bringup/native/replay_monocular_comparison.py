#!/usr/bin/env python3
"""Replay one calibrated camera without ROS transport or changed source stamps."""
import json, re, shutil, struct, subprocess, time
from pathlib import Path
import numpy as np
from rovio_initialization import match_accelerometer_bias_prior

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'logs/zab-offline/livo_comparison'

def run(label):
    inp = OUT/'inputs'/label
    manifest = json.loads((inp/'manifest.json').read_text())
    src = ROOT/'logs'/manifest['source']['run']
    dest = OUT/'rovio'/label
    dest.mkdir(parents=True, exist_ok=True)
    text = (src/'rovio.info').read_text()
    for pattern, value, count in [
        (r'(gyb_[012]\s+)3e-4;', '1e-10', 3),
        (r'(gyb_[012]\s+)3.8e-7;', '1e-16', 3),
        (r'(att_[012]\s+)7.6e-7;', '4e-11', 3),
        (r'(initDepth\s+)0.5;', '0.1', 1),
    ]:
        text, n = re.subn(pattern, rf'\g<1>{value};', text)
        assert n == count
    camera = re.search(r'Camera1\s*\{[^}]+\}', text).group().replace('Camera1','Camera0')
    text = re.sub(r'Camera[012]\s*\{[^}]+\}', '', text) + '\n' + camera + '\n'
    text, bias_variances = match_accelerometer_bias_prior(text, src/'dragon_slam.urdf')
    (dest/'rovio.info').write_text(text)
    shutil.copyfile(src/'cam1.yaml', dest/'cam0.yaml')
    command = ['env','-i','HOME=/root','USER=root','PATH=/usr/local/bin:/usr/bin:/bin','LANG=C.UTF-8',
        'bash','--noprofile','--norc','-c',
        'source /root/uas_native/env_ros1.sh\nsource /root/multilink_autonomy_ws/slam_ros1_ws/devel/setup.bash\n'
        'export ROS_MASTER_URI=http://127.0.0.1:11376\nexec "$@"', 'replay',
        str(OUT/'audit25'),str(dest/'rovio.info'),str(dest),str(dest/'result')]
    start=time.monotonic(); counts=[0,0]
    with (dest/'replay.log').open('w') as log:
        proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT)
        try:
            for chunk in manifest['chunks']:
                with np.load(inp/chunk) as archive:
                    a={k:archive[k] for k in archive.files}
                    for stamp,kind,index,stored in a['order']:
                        if stamp < 2003000000 or kind not in (0,2): continue
                        if kind == 0: payload=struct.pack('<6d',*a['imu'][index]); k=0
                        else: payload=a['cam1_im'][index].tobytes();k=1
                        proc.stdin.write(struct.pack('<IQ', k, int(stamp))+payload);counts[k]+=1
                print(label,chunk,counts,flush=True)
            proc.stdin.close()
            assert proc.wait(timeout=120)==0
        finally:
            if proc.poll() is None: proc.terminate();proc.wait(timeout=10)
    (dest/'summary.json').write_text(json.dumps(dict(camera='link3',counts=counts,wall_s=time.monotonic()-start,source=manifest['source'],window=manifest['window'],initial_accelerometer_bias_variance=bias_variances,initialization_profile='sensor_bias_second_moment',truth_used_for_initialization=False),indent=2))

if __name__=='__main__':
    import sys
    for label in sys.argv[1:]: run(label)
