#!/usr/bin/env python3
"""Summarize a completed straight flight and finalized rosbag metadata."""
import argparse,json
from pathlib import Path
import numpy as np
import yaml

p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();root=args.run
plan=json.loads((root/'route/route.json').read_text());status=json.loads((root/'route_status.json').read_text())
track=np.genfromtxt(root/'route_tracking.csv',delimiter=',',names=True)
flight=track[track['route_t']>0]
a,b=np.array(plan['waypoints']);direction=(b-a)/np.linalg.norm(b-a)
xyz=np.column_stack([flight['gt_x'],flight['gt_y'],flight['gt_z']])
progress=(xyz-a)@direction
cross=xyz-a-progress[:,None]*direction
truth=np.genfromtxt(root/'truth_raw.csv',delimiter=',',names=True)
cruise=flight[(flight['route_t']>=2)&(flight['route_t']<=100)]
selection=(truth['t']>=cruise['sim_t'][0])&(truth['t']<=cruise['sim_t'][-1]);t=truth['t'][selection]
positions=np.column_stack([truth[k][selection] for k in ('x','y','z')])
# Average measured velocity over 0.5 sim second intervals (not noisy 10 ms differences).
centers=np.arange(t[0],t[-1]-.5,.5)
speeds=[]
for stamp in centers:
 q0=np.array([np.interp(stamp,t,positions[:,i]) for i in range(3)])
 q1=np.array([np.interp(stamp+.5,t,positions[:,i]) for i in range(3)])
 speeds.append(np.linalg.norm(q1-q0)/.5)
meta=yaml.safe_load((root/'rosbag/metadata.yaml').read_text())['rosbag2_bagfile_information']
topics={x['topic_metadata']['name']:x['message_count'] for x in meta['topics_with_message_count']}
report=dict(completed=status['stage']=='complete' and status['flight_state']==0 and status['failure'] is None,
 planned_distance_m=plan['length_m'],final_progress_m=float(progress[-1]),
 max_cross_track_m=float(np.linalg.norm(cross,axis=1).max()),
 max_tracking_error_m=float(flight['tracking_error_m'].max()),
 cruise_speed_mean_mps=float(np.mean(speeds)),cruise_speed_min_mps=float(np.min(speeds)),cruise_speed_max_mps=float(np.max(speeds)),
 final_state=status,bag_duration_s=meta['duration']['nanoseconds']/1e9,
 bag_bytes=sum((root/'rosbag'/f).stat().st_size for f in meta['relative_file_paths']),
 bag_file_count=len(meta['relative_file_paths']),topic_message_counts=topics)
required=['/clock','/dragon/ground_truth','/dragon/imu/raw','/dragon/lidar/points','/slam/odometry','/rovio/odometry']+[f'/dragon/cam_link{i}/image' for i in (1,3,4)]
report['required_topics_recorded']=all(topics.get(t,0)>0 for t in required)
(root/'session_summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
