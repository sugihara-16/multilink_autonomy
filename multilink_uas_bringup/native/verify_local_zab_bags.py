#!/usr/bin/env python3
"""Open every downloaded MCAP and each ROS2 bag directory without publishing data."""
import json,time
from pathlib import Path
import yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path(__file__).resolve().parent/'logs/zab-local-bags'
LABELS=['run1_prefix','run1_tail_a1','run1_tail_b1']
checks={};start=time.monotonic()
for label in LABELS:
 d=ROOT/label/'rosbag';v=json.loads((d.parent/'verified.json').read_text());m=yaml.safe_load((d/'metadata.yaml').read_text())['rosbag2_bagfile_information'];assert set(m['relative_file_paths'])==set(v['files']);count=0
 for n in m['relative_file_paths']:
  p=d/n;assert p.stat().st_size==v['files'][n]['bytes']
  r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(p),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));assert r.has_next();topic,raw,t=r.read_next();assert raw;count+=1;del r
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(d),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));topics={x.name:x.type for x in r.get_all_topics_and_types()}
 needed=['/dragon/imu/raw','/dragon/lidar/points','/dragon/cam_link1/image','/dragon/cam_link3/image','/dragon/cam_link4/image','/dragon/ground_truth'];assert set(needed)<=set(topics)
 r.set_filter(rosbag2_py.StorageFilter(topics=needed));seen={};iterations=0
 while r.has_next() and len(seen)<len(needed):
  topic,raw,t=r.read_next();msg=deserialize_message(raw,get_message(topics[topic]));seen.setdefault(topic,dict(stored_stamp_ns=t,header_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec));iterations+=1;assert iterations<10000
 assert set(seen)==set(needed);del r
 checks[label]=dict(chunks_opened=count,directory_reader_opened=True,sampled_topics=seen,total_messages=m['message_count'],bytes=sum(x['bytes'] for x in v['files'].values()));print(label,checks[label],flush=True)
(ROOT/'validation.json').write_text(json.dumps(dict(status='complete',checks=checks,wall_s=time.monotonic()-start,checksum='Every original MCAP size and MD5 verified by cache_zab_bags.py; metadata copied from adopted cloud metadata',retention='Persistent; excluded from experiment cleanup'),indent=2))
