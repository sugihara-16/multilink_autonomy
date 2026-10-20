#!/usr/bin/env python3
"""Read only CDR Header stamps from finalized raw-sensor bags."""
import argparse,json,sqlite3,struct
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('bag',type=Path);args=p.parse_args();meta=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
expected={'/dragon/imu/raw':.001,'/dragon/lidar/points':.1,**{f'/dragon/cam_link{i}/image':.05 for i in (1,3,4)}}
report={k:dict(count=0,first=None,last=None,max_gap_s=0.,missing_intervals=0,nonmonotonic=0) for k in expected}
for file in meta['relative_file_paths']:
 db=sqlite3.connect(f'file:{args.bag/file}?mode=ro',uri=True)
 topics={name:id for id,name in db.execute('select id,name from topics')}
 for name,period in expected.items():
  if name not in topics:continue
  result=report[name]
  for (header,) in db.execute('select substr(data,1,12) from messages where topic_id=? order by timestamp,id',(topics[name],)):
   sec,nsec=struct.unpack('<iI' if header[1]==1 else '>iI',header[4:12]);t=sec+nsec*1e-9
   if result['first'] is None:result['first']=t
   if result['last'] is not None:
    dt=t-result['last'];result['max_gap_s']=max(result['max_gap_s'],dt)
    if dt<=0:result['nonmonotonic']+=1
    if dt>period*1.5:result['missing_intervals']+=max(1,round(dt/period)-1)
   result['last']=t;result['count']+=1
 db.close()
output=dict(duration_s=meta['duration']['nanoseconds']/1e9,sensors=report,total_messages=meta['message_count'],bytes=sum((args.bag/x).stat().st_size for x in meta['relative_file_paths']))
(args.bag.parent/'raw_bag_validation.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
