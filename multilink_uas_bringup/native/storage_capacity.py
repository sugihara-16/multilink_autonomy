"""Check the host backing a WSL filesystem, not just its virtual capacity."""
import json
import os
import shutil
import subprocess
import time
from pathlib import Path


class StorageCapacity:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.checked = -float('inf')
        self.report = None
        self.host_mount = None

    def read(self):
        now = time.monotonic()
        if now - self.checked < 10:
            return self.report
        report = {'linux_free_bytes': shutil.disk_usage(self.path).free,
                  'host_free_bytes': None, 'error': None}
        if 'microsoft' in Path('/proc/sys/kernel/osrelease').read_text().lower():
            try:
                if self.host_mount is None:
                    distro = os.environ.get('WSL_DISTRO_NAME')
                    if not distro:
                        raise RuntimeError('WSL_DISTRO_NAME is missing')
                    # No credentials are read; return only distro name and backing path.
                    script = ('Get-ItemProperty HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Lxss\\* '
                              '| Select-Object DistributionName,BasePath | ConvertTo-Json -Compress')
                    powershell = shutil.which('powershell.exe') or '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'
                    raw = subprocess.run([powershell, '-NoProfile', '-Command', script],
                                         check=True, capture_output=True, text=True, timeout=8)
                    entries = json.loads(raw.stdout)
                    if isinstance(entries, dict):
                        entries = [entries]
                    base = next(e['BasePath'] for e in entries if e['DistributionName'] == distro)
                    base = base.removeprefix('\\\\?\\')
                    parts = self.path.parts
                    drive = parts[2] if len(parts) > 2 and parts[1] == 'mnt' and len(parts[2]) == 1 else base[0].lower()
                    host_mount = Path('/mnt') / drive
                    if not host_mount.is_mount():
                        raise RuntimeError(f'Host drive is not mounted: {host_mount}')
                    self.host_mount = host_mount
                host_mount = self.host_mount
                drive = host_mount.name
                report['host_drive'] = drive.upper() + ':'
                report['host_free_bytes'] = shutil.disk_usage(host_mount).free
            except (OSError, ValueError, StopIteration, RuntimeError, subprocess.SubprocessError) as exc:
                report['error'] = f'Cannot determine WSL host capacity: {exc}'
        values = [report['linux_free_bytes']]
        if report['host_free_bytes'] is not None:
            values.append(report['host_free_bytes'])
        report['available_bytes'] = min(values) if not report['error'] else None
        self.checked, self.report = now, report
        return report
