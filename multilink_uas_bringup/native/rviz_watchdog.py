#!/usr/bin/env python3
"""Keep the optional RViz UI alive without stopping the recording or flight."""
import argparse,signal,subprocess,time
p=argparse.ArgumentParser();p.add_argument('--config',required=True);args=p.parse_args()
stopping=False

def stop(signum,frame):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
child=None
try:
    while not stopping:
        child=subprocess.Popen(['rviz2','-d',args.config,'--ros-args','-p','use_sim_time:=true'])
        print(f'RViz started: pid={child.pid}',flush=True)
        while not stopping and child.poll() is None:time.sleep(.2)
        if stopping:break
        print(f'RViz exited ({child.returncode}); flight and recording remain active. Restarting.',flush=True)
        deadline=time.monotonic()+2
        while not stopping and time.monotonic()<deadline:time.sleep(.2)
finally:
    if child is not None and child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:child.kill();child.wait()
