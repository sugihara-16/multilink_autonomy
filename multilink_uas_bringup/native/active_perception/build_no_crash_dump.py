#!/usr/bin/env python3
"""Build a process-local core guard; never conceal a failure exit status."""
import hashlib,json,os,subprocess
from pathlib import Path
here=Path(__file__).resolve().parent
out=here.parent.parents[3]/'build/no-crash-dump';out.mkdir(exist_ok=True)
source=here/'performance/no_crash_dump.cpp';lib=out/'libno_crash_dump.so'
subprocess.run(['c++','-O2','-shared','-fPIC',str(source),'-o',str(lib)],check=True)
# Read kernel state from inside a fresh exec, and then deliberately fail. The
# child must remain non-dumpable, while its abnormal exit must still propagate.
code='import ctypes,os,signal; assert ctypes.CDLL(None).prctl(3)==0; os.kill(os.getpid(),signal.SIGSEGV)'
windows=Path('/mnt/c/Users/j-sug/AppData/Local/Temp/wsl-crashes')
before=set(windows.glob('*')) if windows.exists() else set()
r=subprocess.run(['/usr/bin/python3','-c',code],env={**os.environ,'LD_PRELOAD':str(lib)},capture_output=True,text=True,timeout=10)
assert r.returncode==-11,r
after=set(windows.glob('*')) if windows.exists() else set()
assert before==after,'Unexpected Windows dump created'
(out/'manifest.json').write_text(json.dumps(dict(exit_code=r.returncode,new_windows_dumps=0,
    sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,Path(__file__),lib]},
    scope='Process-local PR_SET_DUMPABLE=0 for owned simulations. Exit codes and diagnostic text retained. No machine-wide settings.'),indent=2)+'\n')
print('Non-dumpable after exec; SIGSEGV exit -11 retained; no Windows dump')
