"""Small authenticated range probes against Google's own download endpoints."""
import configparser,json,time,urllib.parse
from pathlib import Path
import requests
out=Path('/root/uas_native/logs/zab-local-bags');c=configparser.ConfigParser();c.read('/root/.config/rclone/rclone.conf');token=json.loads(c['uas_gdrive_private']['token'])['access_token'];fid=json.loads((out/'endpoint_probe_file.json').read_text())['ID'];headers={'Authorization':'Bearer '+token}
r=requests.get('https://www.googleapis.com/drive/v3/files/'+fid,headers=headers,params={'fields':'id,webContentLink'},timeout=20);r.raise_for_status();meta=r.json();results=[]
urls=['https://content.googleapis.com/drive/v3/files/'+fid+'?alt=media']
if meta.get('webContentLink'):urls.append(meta['webContentLink'])
for url in urls:
 start=time.monotonic();got=0
 try:
  with requests.get(url,headers={**headers,'Range':'bytes=0-16777215'},stream=True,timeout=20) as r:
   status=r.status_code;kind=r.headers.get('Content-Type','');host=urllib.parse.urlparse(r.url).hostname
   if status==206:
    for chunk in r.iter_content(1024**2):got+=len(chunk)
   result=dict(host=host,status=status,content_type=kind,bytes=got,wall_s=time.monotonic()-start)
 except requests.RequestException as e:result=dict(host=urllib.parse.urlparse(url).hostname,error=type(e).__name__,bytes=got,wall_s=time.monotonic()-start)
 results.append(result);print(json.dumps(result),flush=True)
(out/'endpoint_probe.json').write_text(json.dumps(results,indent=2))
