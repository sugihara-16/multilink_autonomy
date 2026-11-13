#!/usr/bin/env python3
"""Run the selected holdout owner with freshly built, separately recorded binaries.

The historical selected_gimbal.json is intentionally not rewritten: compiler
outputs on this Linux environment are not claimed to have historical hashes.
"""
import hashlib
import json
import sys
from pathlib import Path
from run_information_holdout_pair import pair_sources, HERE


if __name__ == '__main__':
    source, pacer, patch = pair_sources()
    output = Path(sys.argv[sys.argv.index('--output') + 1])
    # The owner creates output itself; store the build record alongside it.
    record = output.with_name(output.name + '.rebuilt_owner.json')
    if record.exists() or output.exists():
        raise RuntimeError('Fresh output directory required')
    record.parent.mkdir(parents=True, exist_ok=True)
    native = HERE.parent
    workspace = native.parents[3]
    files = list(HERE.glob('*.py')) + list(HERE.glob('*.yaml'))
    files += [p for p in (workspace / 'build').rglob('*')
              if p.is_file() and (p.suffix == '.so' or p.name in
                 ('rovio_gimbal', 'mimosa_batched_node', 'reliable_bridge'))]
    record.write_text(json.dumps(dict(
        historical_manifest_enforced=False,
        reason='Fresh Linux build; source and binary hashes recorded independently',
        generated_owner_sha256=hashlib.sha256(source.encode()).hexdigest(),
        generated_pacer_sha256=hashlib.sha256(pacer.encode()).hexdigest(),
        sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}), indent=2))
    exec(compile(source, str(HERE / 'unipilot_run.py'), 'exec'),
         dict(__name__='__main__', __file__=str(HERE / 'unipilot_run.py'),
              ARRAY_SOURCE=source, ARRAY_PACER=pacer, ARRAY_DIFF=patch))
