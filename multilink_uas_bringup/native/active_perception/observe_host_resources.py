#!/usr/bin/env python3
"""Read-only host storage/memory/GPU observations around one owned flight."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def sample(flight, with_windows):
    mem = {line.split(':')[0]: int(line.split()[1])*1024
           for line in Path('/proc/meminfo').read_text().splitlines()
           if line.startswith(('MemAvailable:', 'MemFree:', 'Cached:', 'SwapFree:', 'SwapTotal:'))}
    own = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            command = (path/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if 'gz sim ' in command and str(flight.resolve()/'world_serial.sdf') in command:
                own.append(dict(pid=int(path.name), rss_bytes=int((path/'statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')))
        except (OSError, ValueError, IndexError):
            continue
    row = dict(wall_time=time.time(), windows_free_bytes=shutil.disk_usage('/mnt/c').free,
               linux_memory=mem, owned_gazebo=own)
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=memory.total,memory.used,utilization.gpu',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=5, check=True)
        row['gpu_total_used_MiB_utilization_percent'] = [int(v.strip()) for v in out.stdout.strip().split(',')]
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        row['gpu_error'] = type(exc).__name__
    if with_windows:
        try:
            script = 'Get-CimInstance Win32_PageFileUsage | Select-Object AllocatedBaseSize,CurrentUsage,PeakUsage | ConvertTo-Json -Compress'
            out = subprocess.run(['/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe',
                                  '-NoProfile', '-NonInteractive', '-Command', script],
                                 capture_output=True, text=True, timeout=10, check=True)
            row['windows_pagefile_MB'] = json.loads(out.stdout)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            row['windows_error'] = type(exc).__name__
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--flight', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--max-wall-s', type=float, default=2700.)
    args = p.parse_args()
    started = time.monotonic()
    with args.output.open('x') as stream:
        index = 0
        while time.monotonic()-started < args.max_wall_s:
            row = sample(args.flight, index % 4 == 0)
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            if index % 4 == 0:
                print('Windows free GB:', round(row['windows_free_bytes']/1e9, 3), flush=True)
            if (args.flight/'runtime.json').exists():
                break
            index += 1
            time.sleep(15)


if __name__ == '__main__':
    main()
