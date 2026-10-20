#!/usr/bin/env python3
"""Replace only the image stream in the existing FAST input; copy IMU/LiDAR exactly."""
import hashlib,json,copy
from pathlib import Path
import numpy as np,rosbag
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'logs/zab-offline/livo_comparison';OUT=ROOT/'logs/zab-offline/fast_camera_comparison'
def images(label,cam):
 d=OUT/'images'/label;m=json.loads((d/'manifest.json').read_text());last=-1
 for chunk in m['chunks']:
  with np.load(d/chunk) as a:
   for t,img in zip(a[f'cam{cam}_t'],a[f'cam{cam}_im']):
    if t<2003000000:continue
    assert int(t)>last;last=int(t);yield int(t),img

def main(label):
 import io
 dest=OUT/'inputs';dest.mkdir(exist_ok=True)
 streams={link:iter(images(label,cam)) for link,cam in [(1,0),(4,2)]}
 paths={link:dest/f'{label}_link{link}.bag' for link in streams}
 assert all(not p.exists() for p in paths.values()),'Refusing to overwrite bags'
 bags={link:rosbag.Bag(str(p),'w',compression='lz4') for link,p in paths.items()};counts={};hashes={t:hashlib.sha256() for t in ['/comparison/imu','/comparison/points']};image_hashes={l:hashlib.sha256() for l in streams}
 try:
  with rosbag.Bag(str(BASE/'inputs'/f'{label}.bag')) as original:
   for topic,msg,t in original.read_messages(topics=['/comparison/imu','/comparison/points','/comparison/image']):
    counts[topic]=counts.get(topic,0)+1
    if topic=='/comparison/image':
     for link,stream in streams.items():
      ts,img=next(stream);assert ts==msg.header.stamp.to_nsec()==t.to_nsec()
      new=copy.deepcopy(msg);new.header.frame_id=f'cam_link{link}_optical';new.data=np.repeat(img[:,:,None],3,axis=2).tobytes();bags[link].write(topic,new,t)
      image_hashes[link].update(str(ts).encode());image_hashes[link].update(new.data)
    else:
     buff=io.BytesIO();msg.serialize(buff);hashes[topic].update(str(t.to_nsec()).encode());hashes[topic].update(buff.getvalue())
     for bag in bags.values():bag.write(topic,msg,t)
  for stream in streams.values():assert next(stream,None) is None
 finally:
  for bag in bags.values():bag.close()
 # Verify serialized non-image payload and topic/message order against originals.
 for link,path in paths.items():
  actual={t:hashlib.sha256() for t in hashes};frames=0;imagehash=hashlib.sha256();order=hashlib.sha256()
  with rosbag.Bag(str(path)) as bag,rosbag.Bag(str(BASE/'inputs'/f'{label}.bag')) as original:
   it=iter(original.read_messages(topics=['/comparison/imu','/comparison/points','/comparison/image']))
   for topic,msg,t in bag.read_messages():
    ot,om,stamp=next(it);assert (topic,t.to_nsec(),msg.header.stamp.to_nsec())==(ot,stamp.to_nsec(),om.header.stamp.to_nsec())
    if topic in actual:
     buff=io.BytesIO();msg.serialize(buff);actual[topic].update(str(t.to_nsec()).encode());actual[topic].update(buff.getvalue())
    else:frames+=1;assert msg.header.frame_id==f'cam_link{link}_optical';imagehash.update(str(t.to_nsec()).encode());imagehash.update(msg.data)
   assert next(it,None) is None
  assert all(actual[t].digest()==hashes[t].digest() for t in hashes) and imagehash.digest()==image_hashes[link].digest()
  (dest/f'{label}_link{link}.json').write_text(json.dumps(dict(link=link,bag=str(path),reference_bag=str(BASE/'inputs'/f'{label}.bag'),counts=counts,imu_lidar_serialized_identical=True,message_order_and_timestamps_identical=True,image_sha256=imagehash.hexdigest(),sensor_sha256={k:v.hexdigest() for k,v in hashes.items()}),indent=2))
 print(label,'bags verified',counts,flush=True)
if __name__=='__main__':
 import sys
 for label in sys.argv[1:]:main(label)
