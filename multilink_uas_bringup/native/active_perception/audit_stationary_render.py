#!/usr/bin/env python3
"""Three read-only render controls at recorded camera poses, no estimators.

Ground truth is used only to reconstruct a past diagnostic view. The original
world, meshes, textures, and running experiment files are never modified.
"""
import argparse,copy,json,os,shlex,shutil,signal,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]
sys.path.insert(0,str(NATIVE))
from manager import CLEAN
from report_routes import load,poses,interp

def record(output):
 import rclpy,cv2
 from rclpy.qos import qos_profile_sensor_data
 from sensor_msgs.msg import Image
 rclpy.init();node=rclpy.create_node('stationary_render_recorder');counts={};latest={};subs=[]
 for name in ['active','fixed']:
  def callback(m,name=name):counts[name]=counts.get(name,0)+1;latest[name]=m
  subs.append(node.create_subscription(Image,'/audit/'+name,callback,qos_profile_sensor_data))
 try:
  end=time.monotonic()+60
  while min([counts.get(n,0) for n in ['active','fixed']])<15 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
  if min([counts.get(n,0) for n in ['active','fixed']])<15:raise RuntimeError('Missing diagnostic image stream: '+str(counts))
  stats={}
  for name,m in latest.items():
   assert m.encoding=='rgb8';a=np.ndarray((m.height,m.width,3),np.uint8,buffer=m.data,strides=(m.step,3,1)).copy()
   cv2.imwrite(str(output/(name+'.png')),cv2.cvtColor(a,cv2.COLOR_RGB2BGR));g=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY)
   c=cv2.goodFeaturesToTrack(g,400,.01,8,blockSize=7)
   stats[name]=dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,count=counts[name],mean=float(g.mean()),std=float(g.std()),bright_fraction=float(np.mean(g>200)),dark_fraction=float(np.mean(g<35)),corners=0 if c is None else len(c))
  (output/'image_stats.json').write_text(json.dumps(stats,indent=2)+'\n');print(json.dumps(stats),flush=True)
 finally:node.destroy_node();rclpy.shutdown()

def main():
 p=argparse.ArgumentParser();p.add_argument('--flight',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--record',action='store_true');p.add_argument('--variants',nargs='+',default=['batch_original','batch_no_shadows','official_original']);p.add_argument('--pin-d3d12',action='store_true');p.add_argument('--core-library',type=Path);p.add_argument('--stop-service',action='store_true');a=p.parse_args()
 if a.record:record(a.output);return
 a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
 m=json.loads((a.flight/'single_view_diagnosis.json').read_text());gt=poses(a.flight/'truth.csv');enc=load(a.flight/'encoders.csv');views={}
 for name in ['active','fixed']:
  stamp=m[name]['stamp_ns'];pos,q=interp(gt,[stamp]);R=q[0].as_matrix();angles=np.array([np.interp(stamp*1e-9,enc['t'],enc[k]) for k in ['yaw','pitch']]) if name=='active' else np.zeros(2)
  origin=pos[0]+R@np.array([.18,0,-.025]);Rc=R@Rotation.from_euler('ZY',angles).as_matrix();views[name]=np.r_[origin,Rotation.from_matrix(Rc).as_euler('xyz')].tolist()
 (a.output/'recorded_poses.json').write_text(json.dumps(dict(scope=__doc__,views=views,source=str(a.flight)),indent=2)+'\n')
 original=ET.parse(a.flight/'world_serial.sdf').getroot()
 original.find('world').set('name','render_audit')
 for label,batch,shadows in [('batch_original',True,True),('batch_no_shadows',True,False),('official_original',False,True),('batch_double_sided',True,True),('batch_green_background',True,True)]:
  if label not in a.variants:continue
  if shutil.disk_usage('/mnt/c').free<8e9:raise RuntimeError('Windows free space below 8 GB')
  out=a.output/label;out.mkdir();root=copy.deepcopy(original);world=root.find('world')
  for e in list(world):
   if e.tag=='gui' or (e.tag=='plugin' and e.get('name') not in ['gz::sim::systems::Physics','gz::sim::systems::UserCommands','gz::sim::systems::Sensors']):world.remove(e)
  sensor=world.find("plugin[@name='gz::sim::systems::Sensors']")
  sensor.set('filename',str(WS/'build/gz-batch-speed/libgz-sim8-sensors-system.so') if batch else '/usr/lib/x86_64-linux-gnu/libgz-sim8-sensors-system.so.8.15.0')
  if label=='batch_green_background':world.find('scene/background').text='.2 .8 .2 1'
  if label=='batch_double_sided':
   for include in list(world.findall('include')):
    source=WS/'assets/zab/models'/include.findtext('uri').removeprefix('model://')/'model.sdf';obj=ET.parse(source).getroot().find('model');obj.find('pose').text=include.findtext('pose','0 0 0 0 0 0')
    if include.find('name') is not None:obj.set('name',include.findtext('name'))
    for mat in obj.findall('.//visual/material'):ET.SubElement(mat,'double_sided').text='true'
    world.remove(include);world.append(obj)
  if not shadows:
   world.find('scene/shadows').text='false'
   for light in world.findall('light'):
    item=light.find('cast_shadows')
    if item is not None:item.text='false'
  model=ET.SubElement(world,'model',{'name':'diagnostic_cameras'});ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',{'name':'link'})
  for name,pose in views.items():
   s=ET.SubElement(link,'sensor',{'name':name,'type':'camera'});ET.SubElement(s,'pose').text=' '.join(map(str,pose));ET.SubElement(s,'update_rate').text='10';ET.SubElement(s,'always_on').text='true';ET.SubElement(s,'topic').text='/audit/'+name
   camera=ET.SubElement(s,'camera');ET.SubElement(camera,'horizontal_fov').text='1.52';im=ET.SubElement(camera,'image')
   for k,v in [('width','480'),('height','270'),('format','R8G8B8')]:ET.SubElement(im,k).text=v
   clip=ET.SubElement(camera,'clip');ET.SubElement(clip,'near').text='.02';ET.SubElement(clip,'far').text='1000'
  path=out/'world.sdf';ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
  env='source /root/uas_native/env_dragon.sh\nexport ROS_DOMAIN_ID=88 GZ_PARTITION=gimbal_render_audit\nexport GZ_SIM_RESOURCE_PATH=/root/multilink_autonomy_ws/assets/zab/models:${GZ_SIM_RESOURCE_PATH:-}\nexport XDG_RUNTIME_DIR=/tmp/uas-gazebo-runtime-0\n'
  fast=''
  if batch:fast=f'export UAS_BATCH_SENSOR_RENDER=1 GZ_RENDERING_PLUGIN_PATH={WS}/build/gz-rendering-speed-install/lib/gz-rendering-8/engine-plugins\nexport LD_LIBRARY_PATH={WS}/build/gz-sensors-speed/lib:{WS}/build/gz-rendering-speed-install/lib:${{LD_LIBRARY_PATH:-}}\n'
  if a.pin_d3d12:fast+='export LD_PRELOAD=/usr/lib/wsl/lib/libd3d12core.so\n'
  if a.core_library:
   if not (a.core_library/'libgz-sim8.so.8').exists():raise RuntimeError('Isolated Gazebo core missing')
   fast+='export LD_LIBRARY_PATH='+shlex.quote(str(a.core_library))+':${LD_LIBRARY_PATH:-}\n'
   fast+='export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}\n'
  processes=[]
  def start(name,cmd,extra=''):
   guard=WS/'build/no-crash-dump/libno_crash_dump.so'
   if guard.exists():extra+='export LD_PRELOAD='+shlex.quote(str(guard))+':${LD_PRELOAD:-}\n'
   with (out/(name+'.log')).open('w') as log:
    proc=subprocess.Popen(['bash','--noprofile','--norc','-c',env+extra+'exec '+shlex.join(map(str,cmd))],env=CLEAN,cwd=NATIVE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   processes.append((name,proc));(out/'process_ids.json').write_text(json.dumps({n:p.pid for n,p in processes},indent=2)+'\n');return proc
  try:
   start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',*[f'/audit/{n}@sensor_msgs/msg/Image[gz.msgs.Image' for n in views]])
   reader=start('capture',['python3',HERE/'audit_stationary_render.py','--record','--output',out])
   sim=start('simulation',['gz','sim','-s','-r','--render-engine','ogre2','--render-engine-api-backend','opengl',path],fast)
   deadline=time.monotonic()+75
   while reader.poll() is None and time.monotonic()<deadline:
    if shutil.disk_usage('/mnt/c').free<8e9:raise RuntimeError('Windows free space below 8 GB')
    time.sleep(.5)
   (out/'live_maps.txt').write_text(Path(f'/proc/{sim.pid}/maps').read_text())
   if reader.poll() is None:raise RuntimeError('Capture timeout')
   if reader.returncode:raise RuntimeError('Capture failed: '+str(out))
   print('CAPTURED',label,(out/'image_stats.json').read_text(),flush=True)
  finally:
   previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
   if a.stop_service and any(n=='simulation' and p.poll() is None for n,p in processes):
    stopper=start('stop_service',['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'])
    try:stopper.wait(timeout=4)
    except subprocess.TimeoutExpired:pass
    try:sim.wait(timeout=5)
    except subprocess.TimeoutExpired:pass
   for name,proc in processes:
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
   for name,proc in processes:
    try:proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
     os.killpg(proc.pid,signal.SIGTERM)
     try:proc.wait(timeout=10)
     except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
   (out/'process_exit.json').write_text(json.dumps({name:p.returncode for name,p in processes},indent=2)+'\n')
   signal.signal(signal.SIGINT,previous)
if __name__=='__main__':main()
