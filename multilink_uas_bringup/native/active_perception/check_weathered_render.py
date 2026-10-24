#!/usr/bin/env python3
"""Inspect procedural weathered tunnel views without flying or recording bags."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np
from PIL import Image as PILImage,ImageDraw
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

VIEWS={'yaw_m50':(-50,0),'front':(0,0),'yaw_p50':(50,0),'pitch_p90':(0,90),'pitch_m90':(0,-90)}

def main():
    p=argparse.ArgumentParser();p.add_argument('assets',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    os.environ['GZ_PARTITION']='weathered_inspection';os.environ['XDG_RUNTIME_DIR']='/tmp/uas-gazebo-runtime-0'
    Path(os.environ['XDG_RUNTIME_DIR']).mkdir(exist_ok=True,mode=0o700)
    node=Node();results={}
    positions=[c['x_m'] for c in json.loads((a.assets/'distribution.json').read_text())['clusters']]
    for layout, entry in json.loads((a.assets/'catalog.json').read_text()).items():
        tree=ET.parse(entry['world']);world=tree.getroot().find('world');frames={};callbacks=[]
        for i,x in enumerate(positions):
            for name,(yaw,pitch) in VIEWS.items():
                key=f'{layout}_{i}_{name}';topic='/inspection/'+key
                model=ET.SubElement(world,'model',name=key);ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='camera')
                sensor=ET.SubElement(link,'sensor',name='camera',type='camera');ET.SubElement(sensor,'pose').text=f'{x} 0 2.0 0 {np.deg2rad(pitch)} {np.deg2rad(yaw)}';ET.SubElement(sensor,'topic').text=topic;ET.SubElement(sensor,'update_rate').text='1';ET.SubElement(sensor,'always_on').text='true'
                cam=ET.SubElement(sensor,'camera');ET.SubElement(cam,'horizontal_fov').text=str(2*np.arctan(240/252.51157996671412));image=ET.SubElement(cam,'image');ET.SubElement(image,'width').text='480';ET.SubElement(image,'height').text='270';ET.SubElement(image,'format').text='R8G8B8';clip=ET.SubElement(cam,'clip');ET.SubElement(clip,'near').text='.05';ET.SubElement(clip,'far').text='1000'
                def callback(msg,key=key):
                    if key in frames:return
                    rgb=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy();frames[key]=rgb
                callbacks.append(callback);node.subscribe(Image,topic,callback)
        path=a.output/f'{layout}_inspection.sdf';tree.write(path,encoding='unicode')
        env=os.environ.copy();env['LD_PRELOAD']='/usr/lib/wsl/lib/libd3d12core.so';started=time.monotonic()
        with (a.output/f'{layout}_gazebo.log').open('w') as log:
            proc=subprocess.Popen(['gz','sim','-s','-r','--render-engine','ogre2',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while len(frames)<len(positions)*5:
                    if proc.poll() is not None:raise RuntimeError('Gazebo exited: '+layout)
                    if time.monotonic()-started>90:raise RuntimeError(f'Images missing: {layout} {len(frames)}/{len(positions)*5}')
                    time.sleep(.2)
            finally:
                if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        canvas=PILImage.new('RGB',(1600,len(positions)*270),'white');draw=ImageDraw.Draw(canvas);records={}
        for i,x in enumerate(positions):
            for j,name in enumerate(VIEWS):
                key=f'{layout}_{i}_{name}';rgb=frames[key];PILImage.fromarray(rgb).save(a.output/f'{key}.png');gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
                n=len(cv2.FastFeatureDetector_create(threshold=20,nonmaxSuppression=True).detect(gray))
                records[key]=dict(x_m=x,view=name,corners=n,std_intensity=float(gray.std()),mean_intensity=float(gray.mean()))
                canvas.paste(PILImage.fromarray(rgb).resize((320,240)),(j*320,i*270+30));draw.text((j*320+4,i*270+5),f'x={x:.1f} {name}: {n}',fill='black')
        canvas.save(a.output/f'{layout}_contact_sheet.jpg');results[layout]=records
        (a.output/'results.json').write_text(json.dumps(results,indent=2)+'\n');print(layout,{k:v['corners'] for k,v in records.items()},flush=True)

if __name__=='__main__':main()
