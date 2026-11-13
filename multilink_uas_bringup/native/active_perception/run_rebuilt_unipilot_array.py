#!/usr/bin/env python3
"""Fresh Linux build, six simultaneous independent monocular streams."""
import difflib
import hashlib
import json
import sys
from pathlib import Path
import yaml
from run_information_ablation_array import ablation_sources, HERE, VIEWS


if __name__ == '__main__':
    cfg = yaml.safe_load(Path(sys.argv[sys.argv.index('--planner-config') + 1]).read_text())
    assert cfg['view_information_mode'] == 'lidar_conditioned'
    assert cfg['common_preparation_cycles'] == 2 and cfg['hold_active_view_in_second_cycle']
    source, pacer, _ = ablation_sources()
    old_binary="WS/'build/rovio-joint-geometry/rovio_gimbal'"
    assert source.count(old_binary) == 1
    source=source.replace(old_binary,"WS/'build/rovio-lossless-output/rovio_gimbal'")
    assert len(VIEWS) == 4 and 'VIEWS.clear()' not in source
    patch = ''.join(difflib.unified_diff(
        (HERE/'unipilot_run.py').read_text().splitlines(True), source.splitlines(True),
        fromfile='unipilot_run.py', tofile='effective_array_owner.py'))
    output = Path(sys.argv[sys.argv.index('--output') + 1])
    record = output.with_name(output.name + '.rebuilt_owner.json')
    if record.exists() or output.exists():
        raise RuntimeError('Fresh output directory required')
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text(json.dumps(dict(
        historical_manifest_enforced=False,
        reason='Fresh Linux build; campaign protocol records source and binary hashes',
        generated_owner_sha256=hashlib.sha256(source.encode()).hexdigest(),
        generated_pacer_sha256=hashlib.sha256(pacer.encode()).hexdigest(),
        fixed_views=['front', *VIEWS], independent_monocular_streams=6), indent=2))
    for name in VIEWS:
        sys.argv.extend(['--pacing-extra-lidar', '/array_'+name+'_mimosa/lidar/manager/debug'])
    exec(compile(source, str(HERE/'unipilot_run.py'), 'exec'),
         dict(__name__='__main__', __file__=str(HERE/'unipilot_run.py'),
              ARRAY_SOURCE=source, ARRAY_PACER=pacer, ARRAY_DIFF=patch))
