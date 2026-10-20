#!/usr/bin/env python3
"""Build opt-in scheduling optimizations without replacing system libraries."""
import argparse,json,shlex,subprocess
from pathlib import Path

HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]
BUILD=WS/'build'
PACKAGES={
 'gz-rendering':('gz-rendering8_8.2.3','f1249da6e07f0d6b7adaba28109fa7af727fd322','gz-rendering-speed','gz-rendering8.patch'),
 'gz-sensors':('gz-sensors8_8.2.2','9348f9fe8a11b9d50381819f51e17e136aedab8a','gz-sensors-speed','gz-sensors8.patch'),
 'gz-sim':('gz-sim8_8.15.0','446a44335a45b704b4d36dabcc5508ee34eeb3d8','gz-speed','gz-sim8.patch')}

def execute(cmd,log=None,cwd=None):
    print(shlex.join(map(str,cmd)),flush=True)
    if log:
        with log.open('w') as f:subprocess.run(list(map(str,cmd)),cwd=cwd,stdout=f,stderr=subprocess.STDOUT,check=True)
    else:subprocess.run(list(map(str,cmd)),cwd=cwd,check=True)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--jobs',type=int,default=6);a=parser.parse_args()
    manifest={}
    for package,(tag,revision,directory,patchname) in PACKAGES.items():
        source=BUILD/(directory+'-source');build=BUILD/directory;build.mkdir(exist_ok=True,parents=True)
        if not source.exists():execute(['git','clone','--depth','1','--branch',tag,f'https://github.com/gazebosim/{package}.git',source])
        actual=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
        if actual!=revision:raise RuntimeError(f'Unexpected {package} revision: {actual}')
        patch=HERE/'performance'/patchname
        current=subprocess.check_output(['git','-C',str(source),'diff'])
        if not current:execute(['git','-C',source,'apply',patch])
        elif current!=patch.read_bytes():raise RuntimeError(f'Unexpected local changes: {source}')
        manifest[package]=dict(revision=revision,patch=str(patch))
        execute(['cmake','-S',source,'-B',build,'-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF',
            f'-DCMAKE_INSTALL_PREFIX={build}-install',*(['-DSKIP_ogre=ON'] if package=='gz-rendering' else [])],build/'configure.log',cwd=build)
    render=BUILD/'gz-rendering-speed'
    execute(['cmake','--build',render,'--target','gz-rendering8-ogre2','-j',str(a.jobs)],render/'build.log')
    link=render/'libgz-rendering-ogre2.so'
    if not link.is_symlink():link.symlink_to('libgz-rendering8-ogre2.so')
    execute(['cmake','--install',render],render/'install.log')
    sensors=BUILD/'gz-sensors-speed'
    execute(['cmake','--build',sensors,'--target','gz-sensors8-rendering','-j',str(a.jobs)],sensors/'build.log')
    base=BUILD/'gz-speed/src/systems/sensors/CMakeFiles/gz-sim8-sensors-system.dir'
    flags={line.split(' = ')[0]:shlex.split(line.split(' = ')[1]) for line in (base/'flags.make').read_text().splitlines() if ' = ' in line}
    out=BUILD/'gz-batch-speed';out.mkdir(exist_ok=True);obj=out/'Sensors.o'
    execute(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',
        BUILD/'gz-speed-source/src/systems/sensors/Sensors.cc','-o',obj],out/'compile.log')
    command=shlex.split((base/'link.txt').read_text());command[command.index('-o')+1]=str(out/'libgz-sim8-sensors-system.so.8.15.0')
    for i,item in enumerate(command):
        if item.endswith('Sensors.cc.o'):command[i]=str(obj)
        elif item.startswith('../../../lib/'):command[i]='/usr/lib/x86_64-linux-gnu/'+Path(item).name
        elif 'libgz-sensors8-rendering.so' in item:command[i]=str(sensors/'lib/libgz-sensors8-rendering.so')
    execute(command,out/'link.log')
    for name in ('libgz-sim8-sensors-system.so','libgz-sim8-sensors-system.so.8','libgz-sim-sensors-system.so'):
        p=out/name
        if p.is_symlink():p.unlink()
        p.symlink_to('libgz-sim8-sensors-system.so.8.15.0')
    (out/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
