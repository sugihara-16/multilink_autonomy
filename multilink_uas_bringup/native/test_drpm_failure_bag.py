"""Omitting unused images must preserve all actual MIMOSA inputs byte for byte."""
import hashlib,io,json,shutil,struct
from pathlib import Path
import rosbag
import make_comparison_bag as b
root=Path(__file__).resolve().parent;out=root/'logs/zab-offline/drpm_fast_failures';d=out/'inputs/integration_preview';d.mkdir(exist_ok=True)
s=json.loads((out/'rovio/integration_preview/summary.json').read_text());names=['rosbag_0.npz','rosbag_1.npz','rosbag_2.npz']
for n in names:
 p=d/n
 if not p.exists():p.symlink_to(out/'inputs/run1_prefix'/n)
(d/'manifest.json').write_text(json.dumps(dict(source=s['source'],window=s['window'],chunks=names)))
b.OUT=out;results=[]
for include in [True,False]:
 b.main('integration_preview',include_images=include);p=out/'inputs/integration_preview.bag';h=hashlib.sha256();counts={}
 with rosbag.Bag(str(p)) as bag:
  for topic,m,t in bag.read_messages():
   counts[topic]=counts.get(topic,0)+1
   if topic=='/comparison/image':continue
   z=io.BytesIO();m.serialize(z);h.update(topic.encode()+struct.pack('<Q',t.to_nsec())+z.getvalue())
 results.append(dict(images_included=include,bytes=p.stat().st_size,counts=counts,input_sha256=h.hexdigest()))
 p.unlink()
assert results[0]['input_sha256']==results[1]['input_sha256']
assert '/comparison/image' not in results[1]['counts']
(out/'bag_image_omission_test.json').write_text(json.dumps(dict(passed=True,results=results),indent=2));shutil.rmtree(d)
print(json.dumps(results),flush=True)
