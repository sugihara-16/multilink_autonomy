#!/usr/bin/env python3
"""Combine already verified synchronized images; never retime or duplicate LiDAR/IMU."""
import copy,hashlib,io,json
from pathlib import Path
import rosbag
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';BASE=LOG/'livo_comparison';CAMS=LOG/'fast_camera_comparison';OUT=LOG/'fast_multicamera'
def main(label):
 OUT.mkdir(exist_ok=True);dest=OUT/'inputs';dest.mkdir(exist_ok=True);path=dest/f'{label}.bag'
 if path.exists():raise RuntimeError('Refusing to overwrite '+str(path))
 sensor_hash=hashlib.sha256();counts={};stamp_hash={i:hashlib.sha256() for i in (1,3,4)}
 with rosbag.Bag(str(BASE/'inputs'/f'{label}.bag')) as reference,rosbag.Bag(str(CAMS/'inputs'/f'{label}_link1.bag')) as b1,rosbag.Bag(str(CAMS/'inputs'/f'{label}_link4.bag')) as b4,rosbag.Bag(str(path),'w',compression='lz4') as output:
  its={1:iter(b1.read_messages(topics=['/comparison/image'])),4:iter(b4.read_messages(topics=['/comparison/image']))}
  for topic,msg,t in reference.read_messages(topics=['/comparison/imu','/comparison/points','/comparison/image']):
   if topic=='/comparison/image':
    for i in (1,3,4):
     if i==3:image=copy.deepcopy(msg);image.header.frame_id='cam_link3_optical'
     else:
      _,image,it=next(its[i]);assert it==t and image.header.stamp==msg.header.stamp
     ot=f'/comparison/image/link{i}';output.write(ot,image,t);counts[ot]=counts.get(ot,0)+1;stamp_hash[i].update(str(t.to_nsec()).encode())
   else:
    output.write(topic,msg,t);counts[topic]=counts.get(topic,0)+1;buff=io.BytesIO();msg.serialize(buff);sensor_hash.update(str(t.to_nsec()).encode());sensor_hash.update(buff.getvalue())
  assert all(next(it,None) is None for it in its.values())
 actual=hashlib.sha256();actual_counts={};actual_stamps={i:hashlib.sha256() for i in stamp_hash}
 with rosbag.Bag(str(path)) as bag:
  for topic,msg,t in bag.read_messages():
   assert t==msg.header.stamp
   actual_counts[topic]=actual_counts.get(topic,0)+1
   if topic.startswith('/comparison/image/link'):actual_stamps[int(topic[-1])].update(str(t.to_nsec()).encode())
   else:
    buff=io.BytesIO();msg.serialize(buff);actual.update(str(t.to_nsec()).encode());actual.update(buff.getvalue())
 assert actual.digest()==sensor_hash.digest() and actual_counts==counts
 assert len({h.hexdigest() for h in actual_stamps.values()}|{h.hexdigest() for h in stamp_hash.values()})==1
 (dest/f'{label}.json').write_text(json.dumps(dict(counts=counts,sensor_hash=sensor_hash.hexdigest(),identical_image_stamps=True,unchanged_imu_lidar=True),indent=2));print(label,counts,flush=True)
if __name__=='__main__':
 import sys
 for label in sys.argv[1:] or ['run2a_end','run1_tail_a2_prefix']:main(label)
