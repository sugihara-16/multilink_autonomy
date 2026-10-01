#!/usr/bin/env python3
import concurrent.futures, pathlib, urllib.request, tarfile, io, yaml, time
root=pathlib.Path('/root/ros1_noetic/src'); root.mkdir(parents=True,exist_ok=True)
manifest=yaml.safe_load(pathlib.Path(__file__).with_name('noetic.rosinstall').read_text())
def fetch(item):
 s=item['tar']; dest=root/s['local-name']
 if (dest/'package.xml').exists(): return
 url=s['uri'].replace('https://github.com/','https://codeload.github.com/').replace('/archive/','/tar.gz/')
 if url.endswith('.tar.gz'):url=url[:-7]
 for i in range(4):
  try:
   data=urllib.request.urlopen(url,timeout=90).read();break
  except Exception:
   if i==3: raise
   time.sleep(2)
 dest.mkdir(parents=True,exist_ok=True)
 with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as t:
  for m in t.getmembers():
   parts=pathlib.PurePosixPath(m.name).parts[1:]
   if not parts:continue
   if '..' in parts or m.issym() or m.islnk():raise RuntimeError('unsafe archive member '+m.name)
   m.name='/'.join(parts); t.extract(m,dest)
 print(s['local-name'],flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
 list(pool.map(fetch,manifest))
