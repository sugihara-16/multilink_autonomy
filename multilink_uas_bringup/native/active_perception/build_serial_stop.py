#!/usr/bin/env python3
"""Build an isolated Gazebo 8.15.0 core with serialized shutdown only."""
import hashlib,json,shutil,subprocess
from pathlib import Path
here=Path(__file__).resolve().parent;ws=here.parent.parents[3]
up=ws/'build/gz-speed-source';src=ws/'build/gz-stop-source';out=ws/'build/gz-stop'
revision='446a44335a45b704b4d36dabcc5508ee34eeb3d8'
assert subprocess.check_output(['git','-C',str(up),'rev-parse','HEAD'],text=True).strip()==revision
def official(name):return subprocess.check_output(['git','-C',str(up),'show','HEAD:'+name]).decode()
if not src.exists():
    shutil.copytree(up,src,ignore=shutil.ignore_patterns('.git'))
    for name in subprocess.check_output(['git','-C',str(up),'diff','--name-only'],text=True).splitlines():
        (src/name).write_text(official(name))
patches={}
for name,old,new in [
    ('src/ServerPrivate.hh','      public: std::mutex runMutex;',
     '      public: std::mutex runMutex;\n\n      // Serialize repeated shutdown from the service, signal handler and destructor.\n      private: std::mutex stopMutex;'),
    ('src/ServerPrivate.cc','void ServerPrivate::Stop()\n{\n  this->running = false;',
     'void ServerPrivate::Stop()\n{\n  std::lock_guard<std::mutex> lock(this->stopMutex);\n  this->running = false;')]:
    original=official(name);assert original.count(old)==1
    patched=original.replace(old,new);p=src/name
    if p.read_text()!=patched:p.write_text(patched)
    patches[name]=dict(upstream_sha256=hashlib.sha256(original.encode()).hexdigest(),patched_sha256=hashlib.sha256(patched.encode()).hexdigest())
subprocess.run(['cmake','-S',str(src),'-B',str(out),'-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF',f'-DCMAKE_INSTALL_PREFIX={ws}/build/gz-stop-install'],check=True)
subprocess.run(['cmake','--build',str(out),'--target','gz-sim8','-j2'],check=True)
lib=out/'lib/libgz-sim8.so.8.15.0'
(out/'manifest.json').write_text(json.dumps(dict(upstream_revision=revision,changes=patches,library=str(lib),
    binary_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    scope='Isolated Gazebo 8.15.0 core. Serialize concurrent ServerPrivate::Stop calls; no simulation-step, rendering, sensor or physics changes.'),indent=2)+'\n')
