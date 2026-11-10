"""Serial, finite paired DRAGON experiment; retain every outcome and stop children."""
import argparse, hashlib, json, os, signal, shutil, subprocess, sys, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
NATIVE=HERE.parent
WS=NATIVE.parents[3]

def capture_loaded(out, port):
    if (out/'loaded_fusion_library.json').exists():return
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            cmd=(proc/'cmdline').read_bytes().split(b'\0')
            if not cmd or not cmd[0].endswith(b'/mimosa_batched_node'):continue
            env=(proc/'environ').read_bytes().split(b'\0')
            if f'ROS_MASTER_URI=http://127.0.0.1:{port}'.encode() not in env:continue
            maps=[l for l in (proc/'maps').read_text().splitlines() if 'libmimosa_lib.so' in l]
            if maps:(out/'loaded_fusion_library.json').write_text(json.dumps(dict(pid=int(proc.name),maps=maps),indent=2))
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue

def usable_flight(out, keep_speed_violations=False):
    if not (out/'completion.json').exists() or not (out/'validation.json').exists():return False
    done=json.loads((out/'completion.json').read_text());valid=json.loads((out/'validation.json').read_text())
    checks={k:v for k,v in valid['checks'].items() if not (keep_speed_violations and k=='joint_slew')}
    return done['flight_completed'] and bool(checks) and all(checks.values())


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--keep-speed-violations',action='store_true',help='Retain measured completed flights with a failed speed audit; keep the failed flag in the report');a=p.parse_args()
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=True)
    seeds=[20262801,20262802,20262803]
    tasks=[(mechanism,seed) for i,seed in enumerate(seeds) for mechanism in (('joint','gimbal') if i%2==0 else ('gimbal','joint'))]
    files=list(HERE.glob('*.py'))+[NATIVE/x for x in ('dragon_articulated.py','dragon_gimbal.py','dragon_sim.launch.py')]
    files += [WS/'build/rovio-articulated/rovio_articulated',WS/'build/mimosa-batched-vision/mimosa_batched_node',WS/'build/mimosa-batched-drpm-support-view/libmimosa_lib.so']
    frozen={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
    protocol=dict(keep_speed_violations=a.keep_speed_violations,seeds=seeds,distance_m=90,speed_mps=.5,layout=24301,order=tasks,sha256=frozen,
        common='Forward initial mount; physical sensors; full SE3 ROVIO; corrected DRPM20; articulated_planner scoring; body preparation and pacing',
        treatment='Six body joints vs two optical-centre gimbal axes; candidate reachability/kinematics/limits and resulting dynamics differ',
        independence='Independent simulated sensor realizations, not identical raw inputs; requested seed paired',
        historical='Previous corrected gimbal used v10 and rotation-only ROVIO. It is NOT the symmetric comparator here.',
        truth_usage='Flight control and offline evaluation only',record_bag=False)
    manifest=root/'comparison_protocol.json'
    if manifest.exists():
        old=json.loads(manifest.read_text())
        if old!=json.loads(json.dumps(protocol)):raise RuntimeError('Frozen protocol changed; use a new output directory')
    else:manifest.write_text(json.dumps(protocol,indent=2))
    results=[];child=None
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        for i,(mechanism,seed) in enumerate(tasks):
            name=f'{mechanism}_seed{seed}';out=root/name
            if out.exists():
                done=json.loads((out/'completion.json').read_text())
                if not usable_flight(out,a.keep_speed_violations):raise RuntimeError('Existing failed run retained: '+name)
                results.append(dict(name=name,status='already_complete'));continue
            if shutil.disk_usage('/mnt/c').free<12e9:raise RuntimeError('Windows disk reserve below 12 GB')
            changed=[f for f,h in frozen.items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h]
            if changed:raise RuntimeError('Implementation changed: '+str(changed))
            port=11521+i
            cmd=[sys.executable,str(HERE/'run_articulated_dragon.py'),'--output',str(out),'--mechanism-comparison','--fusion-profile','drpm_isotropic20_view','--seed',str(seed),'--distance','90','--speed','.5','--port',str(port),'--domain',str(121+i)]
            if mechanism=='gimbal':cmd+=['--gimbal']
            (root/'status.json').write_text(json.dumps(dict(active=name,completed=results,started_wall=time.time()),indent=2))
            with (root/(name+'.owner.log')).open('w') as log:
                child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
                while child.poll() is None:
                    if out.exists():capture_loaded(out,port)
                    if shutil.disk_usage('/mnt/c').free<8e9:raise RuntimeError('Windows disk reserve below 8 GB')
                    time.sleep(2)
            rc=child.returncode;child=None
            results.append(dict(name=name,returncode=rc))
            if rc and not usable_flight(out,a.keep_speed_violations):raise RuntimeError('Run failed; all results retained: '+name)
            if rc:results[-1]['motion_limit_audit_failed']=True
        subprocess.run([sys.executable,str(HERE/'report_mechanism_comparison.py'),str(root)],check=True)
        (root/'status.json').write_text(json.dumps(dict(status='complete',completed=results),indent=2))
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=45)

if __name__=='__main__':main()
