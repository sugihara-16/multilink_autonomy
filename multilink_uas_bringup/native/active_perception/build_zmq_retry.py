#!/usr/bin/env python3
import subprocess,hashlib,json,os
from pathlib import Path
here=Path(__file__).resolve().parent;out=here.parent.parents[3]/'build/zmq-retry';out.mkdir(exist_ok=True)
source=here/'performance/zmq_eintr_retry.cpp';test=here/'performance/test_zmq_retry.cpp';lib=out/'libzmq_eintr_retry.so'
subprocess.run(['c++','-std=c++17','-O2','-fPIC','-shared',str(source),'-ldl','-o',str(lib)],check=True)
subprocess.run(['c++','-std=c++17','-O2',str(test),'-lzmq','-ldl','-pthread','-o',str(out/'test_zmq_retry')],check=True)
r=subprocess.run([str(out/'test_zmq_retry')],env={**os.environ,'LD_PRELOAD':str(lib)},capture_output=True,text=True,timeout=20,check=True)
(out/'manifest.json').write_text(json.dumps(dict(test_output=r.stdout,test_stderr=r.stderr,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,test,lib]}),indent=2)+'\n');print(r.stdout,r.stderr)
