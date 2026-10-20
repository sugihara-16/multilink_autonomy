"""Audit the actual Gazebo child, rather than just the ROS launch wrapper."""
import re
import os
import signal
import subprocess


def stop_world_clients(processes, timeout=4.):
    """Drain pause/resume requests while the Gazebo service still exists.

    Return actual exit codes, including failures that predate shutdown. A timeout
    remains visible and the owner's final cleanup still owns that process.
    """
    records = []
    for name, process in processes:
        if name != 'pacing':
            continue
        timed_out = False
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
        records.append(dict(name=name, exit_code=process.poll(), timed_out=timed_out))
    return records


def launch_exit_status(text, gazebo_only=False):
    pattern = r'gz-\d+' if gazebo_only else r'[A-Za-z0-9_.-]+-\d+'
    started = re.findall(r'\[(' + pattern + r')\]: process started with pid \[(\d+)\]', text)
    result = []
    for label, pid in started:
        escaped = re.escape(label)
        clean = bool(re.search(r'\[' + escaped + r'\]: process has finished cleanly', text))
        failed = re.findall(r'\[' + escaped + r'\]: process has died .*?exit code (-?\d+)', text)
        result.append(dict(name=label, pid=int(pid), exit_code=int(failed[-1]) if failed else 0 if clean else None))
    return dict(children=result, all_clean=bool(result) and all(r['exit_code'] == 0 for r in result))


def gazebo_exit_status(text):
    return launch_exit_status(text, gazebo_only=True)
