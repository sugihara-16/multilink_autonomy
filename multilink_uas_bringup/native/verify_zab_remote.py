#!/usr/bin/env python3
"""Verify selected campaign files on Drive against local archival receipts."""
import json,subprocess,yaml
from pathlib import Path
from cloud_archive import ROOT,REMOTE,upload,save,file_md5
c=ROOT/"logs/zab-full-campaign"
m=json.loads((c/"dataset_manifest.json").read_text())
files=json.loads(subprocess.check_output(["rclone","lsjson",REMOTE,"--recursive","--files-only","--hash","--tpslimit","2"],text=True))
by={}
for f in files:
 if f["Path"] in by:raise RuntimeError("Duplicate remote path: "+f["Path"])
 by[f["Path"]]=f
checked=0
for entry in m["valid_bags"]:
 r=ROOT/"logs"/entry["run"]
 meta=yaml.safe_load((r/"cloud_metadata.yaml").read_text())["rosbag2_bagfile_information"]
 for filename in meta["relative_file_paths"]:
  receipt=json.loads((r/"cloud_chunks"/(Path(filename).stem+".json")).read_text())
  f=by[r.name+"/rosbag/"+filename]
  assert f["Size"]==receipt["compressed_bytes"] and f["Hashes"]["md5"]==receipt["md5"],f["Path"]
  checked+=1
 for local,remote in [("cloud_metadata.yaml","rosbag/metadata.yaml"),("raw_bag_validation.json","raw_bag_validation.json"),("capture_context.tar.gz","capture_context.tar.gz")]:
  p=r/local;f=by[r.name+"/"+remote]
  assert f["Size"]==p.stat().st_size and f["Hashes"]["md5"]==file_md5(p),f["Path"]
  checked+=1
summary=dict(valid_bags=len(m["valid_bags"]),checked_files=checked,all_size_md5_match=True)
save(c/"remote_final_verification.json",summary)
with (c/"manifest_upload.log").open("a") as log:
 for name in ["dataset_manifest.json","remote_final_verification.json"]:
  upload(c/name,REMOTE+"/"+name,log)
print(json.dumps(summary))
