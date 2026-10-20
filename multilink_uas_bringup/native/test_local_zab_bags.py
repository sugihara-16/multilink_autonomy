"""Check that temporary experiment cleanup preserves persistent sensor originals."""
import hashlib,json,tempfile
from pathlib import Path
import local_zab_bags as local

def main():
 with tempfile.TemporaryDirectory() as tmp:
  root=Path(tmp);local.ROOT=root/'persistent';source=local.ROOT/'run/rosbag/a.mcap';source.parent.mkdir(parents=True);source.write_bytes(b'original sensor bytes');digest=hashlib.md5(source.read_bytes()).hexdigest()
  manifest=local.ROOT/'run/verified.json';manifest.write_text(json.dumps({'files':{'a.mcap':{'md5':digest,'bytes':source.stat().st_size}}}))
  cache=root/'temporary';cache.mkdir();dest=cache/'a.mcap'
  assert local.link_local_chunk('run','a.mcap',dest,digest)
  assert dest.is_symlink() and dest.read_bytes()==source.read_bytes()
  dest.unlink();assert source.read_bytes()==b'original sensor bytes'
  assert not local.link_local_chunk('run','a.mcap',dest,'incorrect hash')
  assert not dest.exists()
  assert not local.link_local_chunk('absent','a.mcap',dest,digest)
  try:local.link_local_chunk('run','a.mcap',source,digest)
  except AssertionError:pass
  else:raise AssertionError('Source overwrite was not blocked')
  assert source.read_bytes()==b'original sensor bytes'
 print('PASS: local reuse, source survives cleanup, hash/source mismatch rejection, original overwrite protection')
if __name__=='__main__':main()
