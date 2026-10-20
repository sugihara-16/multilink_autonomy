#!/usr/bin/env python3
"""Audit output bag topics, exact integer timestamps, CSV agreement, and source frame coverage."""
import csv,json,math
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parent
C=ROOT/'logs/zab-offline'
source=json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())
entries=[]
for item in source['valid_bags']:
 label=item['label'];p=C/label;r=json.loads((p/'result.json').read_text());w=json.loads((p/'window.json').read_text())
 assert r['status']=='complete' and not r['stamp_errors']
 assert not any(set(v)-set(r.get('unavailable_tail_stamps',{}).get(k,[])) for k,v in r['missing_source_stamps'].items()) and not any(r['unexpected_output_stamps'].values())
 expected={k:list(csv.DictReader((p/(k+'.csv')).open())) for k in ('truth','lio','vio')};seen={k:0 for k in expected}
 topics={'/dragon/ground_truth':'truth','/slam/odometry':'lio','/rovio/odometry':'vio'}
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p/'odometry'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 assert {t.name for t in reader.get_all_topics_and_types()}==set(topics)
 while reader.has_next():
  topic,data,stored=reader.read_next();k=topics[topic];msg=deserialize_message(data,Odometry);stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
  row=expected[k][seen[k]];seen[k]+=1
  assert stored==stamp==int(row['stamp_ns']) and w['start_ns']<=stamp<=w['end_ns']
  assert msg.header.stamp.sec==int(row['sec']) and msg.header.stamp.nanosec==int(row['nanosec'])
  assert msg.header.frame_id==row['frame_id'] and msg.child_frame_id==row['child_frame_id']
  pose=msg.pose.pose
  assert [pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]==[float(row[n]) for n in ['x','y','z','qx','qy','qz','qw']]
  v=msg.twist.twist.linear;angular=msg.twist.twist.angular
  assert [v.x,v.y,v.z,angular.x,angular.y,angular.z]==[float(row[n]) for n in ['vx','vy','vz','wx','wy','wz']]
  assert all(math.isfinite(float(row[n])) for n in ['x','y','z','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz'])
 del reader
 assert seen==r['counts']
 entries.append(dict(label=label,source_run=item['run'],window=w,counts=seen,result=r,bag=str(p/'odometry')))
manifest=dict(status='complete',runs=entries,only_odometry=True,timestamp_contract='Original acquisition header stamps retained as integer nanoseconds, also used for rosbag storage timestamps. No zero-shift or synchronization resampling.',initialization='Pre-tracking inputs replayed for estimator initialization; outputs retained only during route tracking.',estimators={'lio':'MIMOSA independent LiDAR+IMU','vio':'ROVIO three cameras+IMU'},coordinate_frames='Native estimator frames preserved; no truth alignment applied.',source_manifest=str(ROOT/'logs/zab-full-campaign/dataset_manifest.json'))
(C/'odometry_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(validated_runs=len(entries),counts={k:sum(e['counts'][k] for e in entries) for k in ('truth','lio','vio')})))
