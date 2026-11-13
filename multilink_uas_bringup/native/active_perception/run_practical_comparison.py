#!/usr/bin/env python3
"""Fresh finite trials of adopted DRAGON configurations and UniPilot reference."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
from run_mechanism_comparison import capture_loaded

HERE = Path(__file__).resolve().parent
NATIVE = HERE.parent
WS = NATIVE.parents[3]


def unipilot_command(out, seed, domain, port, world, distance=90., origin=(5., 0., 2.)):
    cmd = json.loads((NATIVE / 'simulation_inputs/templates/paired_flight.json').read_text())
    cmd[1] = str(HERE / 'run_rebuilt_unipilot.py')
    for flag, values in {
        '--world': [world], '--spawn': [origin[0], origin[1], .4],
        '--route': [world.parent / 'route'], '--route-end': [distance/.5+2],
        '--output': [out], '--seed': [seed], '--domain': [domain],
        '--port': [port], '--duration': [1200],
        '--planner-config': [out.parent / 'unipilot_planner.yaml'],
    }.items():
        i = cmd.index(flag) + 1
        cmd[i:i + len(values)] = list(map(str, values))
    # These observers are absent from the two-stream reference owner.
    while '--pacing-extra-lidar' in cmd:
        i = cmd.index('--pacing-extra-lidar')
        del cmd[i:i + 2]
    return cmd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--arms', nargs='+', choices=['unipilot', 'gimbal', 'joint'],
                        default=['unipilot', 'gimbal', 'joint'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[20262801, 20262802, 20262803])
    parser.add_argument('--distance', type=float, default=90.)
    parser.add_argument('--origin', type=float, nargs=3, default=[5., 0., 2.])
    parser.add_argument('--world', type=Path, default=WS / 'assets/weathered_holdout_20261026/24301/world.sdf')
    parser.add_argument('--domain-base', type=int, default=151)
    parser.add_argument('--port-base', type=int, default=11651)
    parser.add_argument('--continue-failures', action='store_true', help='Retain failed trials and attempt the entire predeclared schedule')
    args = parser.parse_args()
    if 'unipilot' in args.arms:
        route = json.loads((args.world.parent/'route/route.json').read_text())
        if abs(route['length_m']-args.distance)>1e-6 or route['waypoints'][0] != args.origin:
            parser.error('Requested distance/origin must match the UniPilot route')
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((HERE / 'motion_view_v10.yaml').read_text())
    (root / 'unipilot_planner.yaml').write_text(yaml.safe_dump(dict(cfg, view_information_mode='lidar_conditioned')))
    world = args.world.resolve()
    tasks = [(arm, seed) for i, seed in enumerate(args.seeds)
             for arm in (args.arms if i % 2 == 0 else list(reversed(args.arms)))]
    sources = list(HERE.glob('*.py')) + list(HERE.glob('*.yaml')) + list(NATIVE.glob('dragon*.py'))
    sources += [p for p in world.parent.rglob('*') if p.is_file()]
    sources += [WS / 'build' / name for name in (
        'rovio-joint-geometry/rovio_gimbal', 'rovio-articulated/rovio_articulated',
        'mimosa-batched-vision/mimosa_batched_node', 'mimosa-batched-vision/libmimosa_lib.so',
        'mimosa-batched-drpm-support-view/libmimosa_lib.so',
        'reliable-bridge/reliable_bridge', 'native-adapters/native_slam_inputs',
        'native-adapters/native_image_encoder', 'native-adapters/native_monitor',
        'gz-rendering-speed-install/lib/libgz-rendering8.so.8',
        'gz-rendering-speed-install/lib/libgz-sensors8-rendering.so.8',
        'gz-rendering-speed-install/lib/gz-rendering-8/engine-plugins/libgz-rendering-ogre2.so',
        'gz-batch-speed/libgz-sim8-sensors-system.so.8.15.0', 'gz-stop/lib/libgz-sim8.so.8.15.0')]
    sources += list(NATIVE.glob('*launch.py')) + [NATIVE/'env.sh', NATIVE/'env_ros1.sh']
    sources += [Path('/root/ros2/aerial_robot_base_ws/install') / name for name in (
        'spinal/lib/libspinal_flight_control.a',
        'aerial_robot_simulation/lib/libsimulation_attitude_controller.so')]
    frozen = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    protocol = dict(order=tasks, seeds=args.seeds, distance_m=args.distance, speed_mps=.5, origin=args.origin,
                    domain_base=args.domain_base, port_base=args.port_base,
                    world=str(world), sha256=frozen,
                    configurations=dict(gimbal='Adopted v10, rotation-only ROVIO, DRPM20 view correction',
                                        joint='Articulated planner, full camera position/orientation, DRPM20 view correction',
                                        unipilot='Fresh v10 lidar-conditioned holdout reference, DRPM off'),
                    interpretation='Practical configurations; not an isolated mechanism-only experiment',
                    truth_usage='Control and offline evaluation only',
                    retention='All attempted trials retained, no accuracy-based exclusions')
    (root / 'comparison_protocol.json').write_text(json.dumps(protocol, indent=2))
    child = None
    results = []
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        for i, (arm, seed) in enumerate(tasks):
            changed = [p for p, h in frozen.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest() != h]
            if changed:
                raise RuntimeError('Inputs changed during campaign: ' + str(changed))
            name = f'{arm}_seed{seed}'
            out = root / name
            domain, port = args.domain_base + i, args.port_base + i
            if arm == 'unipilot':
                command = unipilot_command(out, seed, domain, port, world, args.distance, args.origin)
            else:
                command = [sys.executable, str(HERE / 'run_articulated_dragon.py'),
                           '--output', str(out), '--seed', str(seed), '--domain', str(domain),
                           '--world', str(world),
                           '--port', str(port), '--distance', str(args.distance), '--speed', '.5',
                           '--fusion-profile', 'drpm_isotropic20_view', '--origin', *map(str, args.origin)]
                command += ['--gimbal', '--unipilot-matched'] if arm == 'gimbal' else ['--mechanism-comparison']
            (root / (name + '.command.json')).write_text(json.dumps(command, indent=2))
            (root / 'status.json').write_text(json.dumps(dict(active=name, completed=results), indent=2))
            print('START', name, flush=True)
            with (root / (name + '.owner.log')).open('w') as log:
                child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                while child.poll() is None:
                    if out.exists():
                        capture_loaded(out, port)
                    time.sleep(2)
            result = dict(name=name, returncode=child.returncode)
            results.append(result)
            child = None
            print('FINISH', result, flush=True)
            # Preserve acquisition failures and stop for diagnosis before more trials.
            if result['returncode'] and not args.continue_failures:
                raise RuntimeError('Trial failed; inspect retained outputs: ' + name)
        (root / 'status.json').write_text(json.dumps(dict(status='complete', completed=results), indent=2))
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=60)
        if len(results) != len(tasks):
            (root / 'status.json').write_text(json.dumps(dict(status='interrupted_or_failed', completed=results), indent=2))


if __name__ == '__main__':
    main()
