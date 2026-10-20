#!/usr/bin/env python3
"""Small live diagnostic: successful feature continuity, without images or poses."""
import argparse,json,time,threading
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=60.);a=p.parse_args()
    rospy.init_node('feature_lifetime_audit',anonymous=True);lock=threading.Lock();start=None;data={k:dict(last=None,open={},durations=[],counts=[],retention=[],frames=0,done=False) for k in ['fixed','active']}
    def update(m,key):
        nonlocal start
        stamp=m.header.stamp.to_sec();ids={int(i) for i,status in point_cloud2.read_points(m,field_names=['id','status'],skip_nans=True) if status==4}
        with lock:
            if start is None:start=stamp
            d=data[key]
            if d['done'] or stamp<start:return
            if stamp>=start+a.seconds:d['done']=True;return
            old=set(d['open']);d['frames']+=1;d['counts'].append(len(ids))
            if old:d['retention'].append(len(old&ids)/len(old))
            for i in old-ids:
                born,censored=d['open'].pop(i)
                if not censored:d['durations'].append(d['last']-born+.05)
            for i in ids-old:d['open'][i]=(stamp,d['frames']==1)
            d['last']=stamp
    subs=[rospy.Subscriber(topic,PointCloud2,lambda m,k=key:update(m,k),queue_size=50,tcp_nodelay=True) for key,topic in [('fixed','/fixed/rovio/pcl'),('active','/rovio/pcl')]]
    wall=time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-wall<120:
        with lock:done=all(d['done'] for d in data.values())
        if done:break
        time.sleep(.1)
    with lock:
        report={'scope':__doc__,'requested_s':a.seconds,'start_sim_s':start,'completed':all(d['done'] for d in data.values()),'sides':{}}
        for k,d in data.items():
            report['sides'][k]={'frames':d['frames'],'last_sim_s':d['last'],'completed_track_segments':len(d['durations']),'right_censored_tracks':len(d['open']),'successful_count_p10_p50_p90':np.quantile(d['counts'],[.1,.5,.9]).tolist() if d['counts'] else [],'mean_adjacent_frame_retention':float(np.mean(d['retention'])) if d['retention'] else None,'completed_duration_p10_p50_p90_s':np.quantile(d['durations'],[.1,.5,.9]).tolist() if d['durations'] else []}
    a.output.parent.mkdir(exist_ok=True,parents=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
