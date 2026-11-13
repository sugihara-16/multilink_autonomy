"""Match installed Harmonic 8.3 sensor/rendering ABI and retain batching patches."""
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

N = Path('/root/uas_native')
sys.path.insert(0, str(N / 'active_perception'))
from build_performance import BUILD, execute

system = Path('/usr/lib/x86_64-linux-gnu')
for package in ('gz-rendering8', 'gz-sensors8'):
    version = subprocess.check_output(['pkg-config', '--modversion', package], text=True).strip()
    if version != '8.3.0':
        raise RuntimeError(f'{package}: expected 8.3.0, got {version}')

# Rendering 8.3.0 already contains the identical inline-worker optimization.
# Keep an isolated copy in the existing runtime location; never modify /usr/lib.
render = BUILD / 'gz-rendering-speed-install'
if render.exists() and not (render / 'system-8.3.json').exists():
    render.rename(BUILD / 'gz-rendering-speed-install.legacy-8.2.3')
lib = render / 'lib'
lib.mkdir(parents=True, exist_ok=True)
copied = []
for path in system.glob('libgz-rendering8*.so*'):
    target = lib / path.name
    if target.is_symlink():target.unlink()
    shutil.copy2(path, target, follow_symlinks=True)
    copied.append(target)
shutil.copytree(system / 'gz-rendering-8/engine-plugins', lib / 'gz-rendering-8/engine-plugins', dirs_exist_ok=True)
(render / 'system-8.3.json').write_text(json.dumps(dict(
    version='8.3.0', source_revision='75563ff4c8c0bf4df4c61fcb5fea231fa1cd4bfa',
    note='The prior Ogre2 worker patch is included upstream in this version.',
    sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in copied}), indent=2))

source = BUILD / 'gz-sensors-8.3-source'
if not source.exists():
    execute(['git', 'clone', '--depth', '1', '--branch', 'gz-sensors8_8.3.0',
             'https://github.com/gazebosim/gz-sensors.git', source])
revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
if revision != '6d74c1fe96d7277c7dec96a54bbc79d5c752e63c':raise RuntimeError('Unexpected sensor source')
patch = N / 'active_perception/performance/gz-sensors8.patch'
if subprocess.run(['git','-C',str(source),'apply','--reverse','--check',str(patch)],capture_output=True).returncode:
    execute(['git','-C',source,'apply',patch])
sensors = BUILD / 'gz-sensors-8.3-speed'
sensors.mkdir(exist_ok=True)
execute(['cmake','-S',source,'-B',sensors,'-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],sensors/'configure.log')
execute(['cmake','--build',sensors,'--target','gz-sensors8-rendering','-j4'],sensors/'build.log')
# Both owners prepend this directory. Resolve the patched sensor library before
# system camera libraries load their dependency with the same SONAME.
for path in (sensors / 'lib').glob('libgz-sensors8-rendering.so*'):
    for directory in (lib, BUILD / 'gz-sensors-speed/lib'):
        target = directory / path.name
        if target.is_symlink():target.unlink()
        shutil.copy2(path, target, follow_symlinks=True)

# Reuse the existing 8.15 simulation patch, relink to the matching sensor ABI.
base = BUILD / 'gz-speed/src/systems/sensors/CMakeFiles/gz-sim8-sensors-system.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (base/'flags.make').read_text().splitlines() if ' = ' in l}
out=BUILD/'gz-batch-speed'
obj=out/'Sensors.o'
execute(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',
         BUILD/'gz-speed-source/src/systems/sensors/Sensors.cc','-o',obj],out/'compile.log')
command=shlex.split((base/'link.txt').read_text())
command[command.index('-o')+1]=str(out/'libgz-sim8-sensors-system.so.8.15.0')
for i,item in enumerate(command):
    if item.endswith('Sensors.cc.o'):command[i]=str(obj)
    elif item.startswith('../../../lib/'):command[i]=str(system/Path(item).name)
    elif 'libgz-sensors8-rendering.so' in item:command[i]=str(sensors/'lib/libgz-sensors8-rendering.so')
execute(command,out/'link.log')
(out/'linux-8.3-manifest.json').write_text(json.dumps(dict(rendering='8.3.0',sensors_revision=revision,
    sensor_patch_sha256=hashlib.sha256(subprocess.check_output(['git','-C',str(source),'diff'])).hexdigest(),
    library_sha256=hashlib.sha256((out/'libgz-sim8-sensors-system.so.8.15.0').read_bytes()).hexdigest()),indent=2))
