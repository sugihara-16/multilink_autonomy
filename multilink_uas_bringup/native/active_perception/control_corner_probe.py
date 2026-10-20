#!/usr/bin/env python3
"""Short, explicitly control-only flight around the failing prefix corner."""
import argparse,json,os,shlex,signal,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]
sys.path.insert(0,str(NATIVE));from manager import CLEAN
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--feedback',type=float,default=0);p.add_argument('--xy-only',action='store_true');p.add_argument('--smooth',action='store_true');p.add_argument('--corner-speed-limit',action='store_true');a=p.parse_args()
a.output.mkdir(exist_ok=False);out=a.output.resolve();route=out/'route';route.mkdir()
source=NATIVE/'logs/speed6-full/run1_prefix_active/route';plan=json.loads((source/'route.json').read_text());segments=json.loads((source/'segments.json').read_text())['segments']
segments=[s for s in segments if s['t0']>=316.14][:2];offset=segments[0]['t0'];position=segments[0]['start']
for s in segments:s['t0']-=offset
plan['waypoints']=[position,segments[-1]['end']]
(route/'route.json').write_text(json.dumps(plan));(route/'segments.json').write_text(json.dumps(dict(segments=segments)))
(out/'protocol.json').write_text(json.dumps(dict(control_only=True,source_route_offset=offset,route_end=22.,velocity_feedback=a.feedback,xy_only=a.xy_only,smooth=a.smooth,corner_speed_limit=a.corner_speed_limit,estimators_running=False)))
children=[]
def start(name,cmd):
 shell=f'source {NATIVE}/env_dragon.sh\nexport ROS_DOMAIN_ID=89 GZ_PARTITION=uas_control_probe\nexport GZ_RENDERING_PLUGIN_PATH={WS}/build/gz-rendering-speed-install/lib/gz-rendering-8/engine-plugins\nexport LD_LIBRARY_PATH={WS}/build/gz-rendering-speed-install/lib:${{LD_LIBRARY_PATH:-}}\nexec '+shlex.join(map(str,cmd))
 log=(out/(name+'.log')).open('w');proc=subprocess.Popen(['bash','--noprofile','--norc','-c',shell],env=CLEAN,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close();children.append(proc);return proc
started=time.monotonic()
try:
 sim=start('simulation',['ros2','launch',NATIVE/'unipilot_gimbal_sim.launch.py','headless:=true','manual_hover:=false','paused:=true','paired:=true','serial_postupdates:=true','batch_render:=true',f'output:={out}',f'spawn_x:={position[0]}',f'spawn_y:={position[1]}',f'spawn_z:={position[2]-1}'])
 deadline=time.monotonic()+30
 while not (out/'unipilot_gimbal.sdf').exists():
  if sim.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Simulation startup failed')
  time.sleep(.1)
 time.sleep(4)
 flight=start('route',['python3',HERE/'unipilot_route.py','--output',out,'--route',route,'--end','22','--airborne-init','--control-only','--velocity-feedback',str(a.feedback),*(['--velocity-feedback-xy-only'] if a.xy_only else []),*(['--smooth-tracking-rate'] if a.smooth else []),*(['--corner-speed-limit'] if a.corner_speed_limit else [])])
 unpause=start('unpause',['gz','service','-s','/world/zab/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false']);unpause.wait(timeout=10)
 while flight.poll() is None:
  if sim.poll() is not None or time.monotonic()-started>180:raise RuntimeError('Control probe timeout/crash')
  time.sleep(.2)
 print((out/'route_status.json').read_text())
finally:
 for proc in reversed(children):
  try:os.killpg(proc.pid,signal.SIGINT)
  except ProcessLookupError:pass
 for proc in reversed(children):
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 (out/'runtime.json').write_text(json.dumps(dict(wall_s=time.monotonic()-started,stopped=True,control_only=True)))
