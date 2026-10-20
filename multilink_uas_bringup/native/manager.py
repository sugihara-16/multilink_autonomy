#!/usr/bin/env python3
"""UAS/DRAGON simulation supervisor. Owns only its recorded child groups."""
import argparse, fcntl, json, os, pathlib, signal, subprocess, sys, time, xmlrpc.client, shlex
ROOT = pathlib.Path(__file__).resolve().parent
STATE = ROOT / 'state.json'
UAS = pathlib.Path('/root/unified_autonomy_stack/workspaces')
SLAM_WS = ROOT.parents[3] / 'slam_ros1_ws'
CLEAN = {k: v for k, v in os.environ.items() if k in ('HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR', 'WSL_INTEROP', 'WSL_DISTRO_NAME', 'WSL2_GUI_APPS_ENABLED')}
CLEAN['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/wsl/lib'

def identity(pid):
    try:
        return pathlib.Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[19]
    except (FileNotFoundError, ProcessLookupError):
        return None

def state():
    try:
        s = json.loads(STATE.read_text())
        return s if identity(s['pid']) == s['starttime'] else None
    except (FileNotFoundError, ValueError):
        return None

def supervise(mode, headless='true', rviz='false', land_after_check='false',
              record='false', world_file=None, spawn=None, flight='route', route_dir=None, once=False):
    if route_dir:
        import hashlib
        route_plan=json.loads((pathlib.Path(route_dir)/'route.json').read_text())
        if not world_file or hashlib.sha256(pathlib.Path(world_file).read_bytes()).hexdigest()!=route_plan['world_sha256']:
            raise RuntimeError('Route world checksum mismatch')
    # One owner for launch and the legacy run.sh entry points alike.
    lock = (ROOT / '.supervisor.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('UAS is already running; stop the existing launch first')
    children = []
    oneshot = set()
    logdir = ROOT / 'logs' / (time.strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}')
    logdir.mkdir(parents=True)
    current = ROOT / 'logs' / 'latest'
    current.unlink(missing_ok=True)
    current.symlink_to(logdir.name)
    status = dict(pid=os.getpid(), starttime=identity(os.getpid()), mode=mode,
                  logs=str(logdir), components={}, record=record == 'true',
                  world_file=world_file, flight=flight)
    STATE.write_text(json.dumps(status, indent=2))
    print(f'UAS {mode}: logs at {logdir}', flush=True)
    def shutdown(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    def launch(name, command, ros1=False, once=False):
        envfile = ROOT / ('env_ros1.sh' if ros1 else ('env_dragon.sh' if mode in ('dragon_slam', 'dragon_vio', 'dragon_compare', 'dragon_record') else 'env.sh'))
        # ROS1 core must be usable before the planner overlay is available too.
        command = f'source {shlex.quote(str(envfile))} && exec {command}'
        out = (logdir / f'{name}.log').open('w')
        p = subprocess.Popen(['bash', '-c', command], cwd=ROOT,
                             stdout=out, stderr=subprocess.STDOUT, start_new_session=True, env=CLEAN)
        out.close()
        children.append(p)
        if once: oneshot.add(p.pid)
        status['components'][name] = p.pid
        STATE.write_text(json.dumps(status, indent=2))
    try:
        if mode in ('stack', 'flight_slam', 'dragon_slam', 'dragon_vio', 'dragon_compare', 'vio_slam'):
            launch('roscore', 'roscore -p 11374', True)
            master = xmlrpc.client.ServerProxy('http://127.0.0.1:11374')
            for _ in range(60):
                try:
                    master.getPid('/uas_native_startup')
                    break
                except OSError:
                    if children[-1].poll() is not None:
                        raise RuntimeError("roscore exited; inspect roscore.log")
                    time.sleep(0.5)
            else:
                raise RuntimeError('roscore failed to start')
            bridge_config = {'flight_slam': 'bridge_slam.yaml', 'dragon_slam': 'bridge_dragon.yaml', 'dragon_vio': 'bridge_dragon_vio.yaml', 'dragon_compare': 'bridge_dragon_vio.yaml', 'vio_slam': 'bridge_vio.yaml'}.get(mode, 'bridge_params.yaml')
            subprocess.run(['bash', '-c', f'source {ROOT}/env_ros1.sh; rosparam load {ROOT}/{bridge_config}; rosparam set /use_sim_time true'],check=True,env=CLEAN)
            launch('bridge', f'bash -c "source {UAS}/ws_ros1_bridge/install/local_setup.bash; exec ros2 run ros1_bridge parameter_bridge"')
            if mode == 'stack':
                launch('planner', f'roslaunch {ROOT}/gbplanner_headless.launch robot_name:=rmf_unipilot config_name:=uav_rl_sim', True)
        model_arg = ''
        if mode == 'flight_slam':
            from prepare_slam_model import prepare_model
            prepare_model(UAS/'ws_sim/src/rmf_gz/resources/robots/rmf_unipilot/model.sdf', logdir/'unipilot_slam.sdf')
            model_arg = f' model_file:={logdir}/unipilot_slam.sdf'
            (logdir/'mimosa').mkdir()
            launch('slam_inputs', f'python3 {ROOT}/slam_inputs.py', True)
            launch('mimosa', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/mimosa_unipilot.launch config:={ROOT}/mimosa_unipilot.yaml logs:={logdir}/mimosa"', True)
        if mode == 'vio_slam':
            from prepare_vio_model import prepare
            prepare(UAS/'ws_sim/src/rmf_gz/resources/robots/rmf_unipilot/model.sdf', logdir)
            model_arg = f' model_file:={logdir}/unipilot_vio.sdf'
            (logdir/'mimosa').mkdir()
            launch('slam_inputs', f'python3 {ROOT}/slam_inputs.py', True)
            launch('vio_images', f'python3 {ROOT}/vio_images.py --calibration {logdir}', True)
            launch('rovio', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/rovio_unipilot.launch calibration:={logdir}"', True)
            launch('mimosa', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/mimosa_unipilot.launch config:={ROOT}/mimosa_vio.yaml logs:={logdir}/mimosa external_odometry:=/rovio/odometry"', True)
            topics = [f'/rmf_unipilot/cam_{name}/image@sensor_msgs/msg/Image[gz.msgs.Image' for name in ['front', 'left', 'right']]
            launch('cameras', shlex.join(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + topics))
            launch('flight', f'python3 {ROOT}/vio_flight.py --output {logdir}')
        if mode in ('dragon_slam', 'dragon_vio', 'dragon_compare', 'dragon_record'):
            if mode != 'dragon_record':
                (logdir/'mimosa').mkdir()
                launch('slam_inputs', f'python3 {ROOT}/slam_inputs.py _imu_topic:=/dragon/imu/raw _points_topic:=/dragon/lidar/points', True)
            vio = mode in ('dragon_vio', 'dragon_compare', 'dragon_record')
            sim_args = [f'headless:={headless}', f'output:={logdir}', f'vio:={str(vio).lower()}']
            if world_file:
                sim_args.append(f'world_file:={world_file}')
            if route_dir:
                sim_args.append(f'takeoff_height:={route_plan["waypoints"][0][2]}')
            if spawn:
                sim_args.extend(f'spawn_{a}:={v}' for a, v in zip('xyz', spawn))
            launch('sim', shlex.join(['ros2', 'launch', str(ROOT/'dragon_sim.launch.py')] + sim_args))
            if world_file:
                launch('controller_ready', f'python3 {ROOT}/zab_controller_ready.py --output {logdir}', once=True)
            if vio:
                for _ in range(120):
                    if (logdir/'camera_manifest.json').exists():
                        break
                    if children[-1].poll() is not None:
                        raise RuntimeError('DRAGON model generation failed; see sim.log')
                    time.sleep(.5)
                else:
                    raise RuntimeError('DRAGON camera calibration generation timed out')
            if vio and mode != 'dragon_record':
                topics = ' '.join(f'/dragon/cam_link{i}/image' for i in (1,3,4))
                launch('vio_images', f'python3 {ROOT}/vio_images.py --calibration {logdir} --topics {topics}', True)
                launch('rovio', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/rovio_unipilot.launch calibration:={logdir}"', True)
            config = 'mimosa_dragon_vio.yaml' if mode == 'dragon_vio' else 'mimosa_dragon.yaml'
            external = ' external_odometry:=/rovio/odometry' if mode == 'dragon_vio' else ''
            if mode != 'dragon_record':
                launch('mimosa', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/mimosa_unipilot.launch config:={ROOT}/{config} logs:={logdir}/mimosa{external}"', True)
            if mode in ('dragon_compare','dragon_record'):
                import shutil
                if flight == 'route':
                    shutil.copytree(pathlib.Path(route_dir) if route_dir else ROOT/'routes/cave_exploration', logdir/'route')
                snapshot = logdir/'implementation'; snapshot.mkdir()
                for pattern in ('*.py', '*.yaml', '*.launch', '*.sh', '*.rviz'):
                    for source in ROOT.glob(pattern): shutil.copy2(source, snapshot/source.name)
                if mode == 'dragon_record':
                    launch('capture_monitor', f'python3 {ROOT}/capture_monitor.py --output {logdir}')
                else:
                    launch('evaluation', f'python3 {ROOT}/odometry_evaluation.py --output {logdir}')
                    launch('error_plot', f'python3 {ROOT}/odometry_error_plot.py --output {logdir}')
                topics = ['/clock', '/dragon/ground_truth', '/dragon/uav/cog/odom',
                    '/dragon/imu/raw', '/dragon/lidar/points', '/dragon/joint_states',
                    '/dragon/flight_state', '/dragon/uav/nav', '/slam/odometry', '/rovio/odometry',
                    '/evaluation/lio/error', '/evaluation/vio/error', '/tf', '/tf_static']
                if mode == 'dragon_record':
                    topics=[t for t in topics if not t.startswith(('/slam/','/rovio/','/evaluation/'))]
                    topics += ['/dragon/uav/baselink/odom']
                topics += [f'/dragon/cam_link{i}/image' for i in (1,3,4)]
                bag_output=logdir/'rosbag'
                if record == 'true' and os.environ.get('UAS_RECORD_SPOOL'):
                    bag_output=pathlib.Path(os.environ['UAS_RECORD_SPOOL'])/logdir.name/'rosbag'
                    bag_output.parent.mkdir(parents=True,exist_ok=True)
                    (logdir/'rosbag').symlink_to(bag_output,target_is_directory=True)
                command = ['ros2','bag','record','-o',str(bag_output),'-b','1073741824',
                    '--use-sim-time','--max-cache-size','104857600',
                    '--qos-profile-overrides-path',str(ROOT/'record_compare_qos.yaml')] + topics
                if record == 'true':
                    launch('recorder', shlex.join(command))
                if flight == 'route':
                    launch('route_flight', f'python3 {ROOT}/dragon_route_flight.py --output {logdir} --route {logdir}/route' + (' --require-recording' if record == 'true' else '') + (' --once' if once else '') + (' --sensors-only' if mode == 'dragon_record' else ''))
            else:
                hover = 'dragon_vio_hover.py' if vio else 'dragon_hover.py'
                launch('hover', f'python3 {ROOT}/{hover} --output {logdir}' + (' --land-after-check' if land_after_check == 'true' else ''))
        else:
            launch('sim', f'ros2 launch {ROOT}/sim_wsl.launch.xml headless:={headless}{model_arg}')
        if mode == 'flight_slam':
            launch('flight', f'python3 {ROOT}/flight_slam.py --output {logdir}')
        if rviz == 'true':
            rviz_config = 'slam.rviz' if mode in ('flight_slam', 'dragon_slam', 'dragon_vio', 'dragon_compare', 'vio_slam') else 'unipilot.rviz'
            if mode == 'vio_slam': rviz_config = 'vio.rviz'
            if mode == 'dragon_vio': rviz_config = 'dragon_vio.rviz'
            if mode == 'dragon_compare': rviz_config = 'dragon_compare.rviz'
            render_env = 'env LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 ' if mode in ('dragon_slam', 'dragon_vio', 'dragon_compare', 'vio_slam') else ''
            if mode == 'dragon_compare':
                launch('rviz', f'{render_env}python3 {ROOT}/rviz_watchdog.py --config {ROOT}/{rviz_config}')
            else:
                launch('rviz', f'{render_env}rviz2 -d {ROOT}/{rviz_config}' + (' --ros-args -p use_sim_time:=true' if mode in ('vio_slam', 'dragon_vio') else ''))
        if mode == 'stack':
            launch('pc_selector', 'ros2 launch robot_bringup cbf_pc_selector_sim.launch.py cfg:=cbf_pc_selector_sim_unipilot.yaml')
            launch('cbf', shlex.join(['ros2', 'run', 'composite_cbf', 'composite_cbf_node', '--ros-args', '-r', '__node:=composite_cbf', '--params-file', str(UAS/'robot_bringup/config/ros2/cbf_sim_unipilot.yaml'), '-p', 'use_sim_time:=true', '-r', '~/obstacles:=/cbf_pc_selector/output_pc', '-r', '~/odom:=/rmf_unipilot/odom', '-r', '~/cmd_in:=/sdf_nmpc/cmd/acc', '-r', '~/safe_cmd_twist:=/validation/cbf_safe_cmd']))
            launch('rl', f'bash -c "source /root/.venvs/uas/bin/activate && cd {UAS}/ws_rl/src/rl_nav/scripts && exec python3 lidar_nav_acc_ros2_sim_node.py --env=lidar_navigation_task --experiment=magpie_accel_policy4"')
        while True:
            for p in children:
                if once and p.pid == status['components'].get('route_flight') and p.poll() == 0:
                    return
                if p.poll() is not None and not (p.pid in oneshot and p.returncode == 0):
                    raise RuntimeError(f'component {p.pid} exited: {p.returncode}; see {logdir}')
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        # A second launch/terminal signal must not interrupt child cleanup.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for p in reversed(children):
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        deadline = time.monotonic() + 15
        while any(p.poll() is None for p in children) and time.monotonic() < deadline:
            time.sleep(0.2)
        for p in reversed(children):
            # Kill the owned group even if its leader exited with descendants alive.
            try: os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError: pass
        for p in children: p.wait()
        STATE.unlink(missing_ok=True)

if __name__ == '__main__':
    mode = sys.argv[1]
    if mode in ('_supervise', 'foreground'):
        parser = argparse.ArgumentParser()
        parser.add_argument('mode', choices=['sim', 'stack', 'flight_slam', 'dragon_slam', 'dragon_vio', 'dragon_compare', 'dragon_record', 'vio_slam'])
        parser.add_argument('--headless', choices=['true', 'false'], default='true')
        parser.add_argument('--rviz', choices=['true', 'false'], default='false')
        parser.add_argument('--land-after-check', choices=['true', 'false'], default='false')
        parser.add_argument('--record', choices=['true', 'false'], default='false')
        parser.add_argument('--world-file')
        parser.add_argument('--spawn', nargs=3, type=float)
        parser.add_argument('--flight', choices=['route', 'none'], default='route')
        parser.add_argument('--route-dir')
        parser.add_argument('--once',action='store_true')
        args = parser.parse_args(sys.argv[2:])
        if args.world_file and args.mode == 'dragon_compare' and args.flight == 'route' and not args.route_dir:
            parser.error('The cave route cannot be used with a different world; specify --flight none')
        supervise(args.mode, args.headless, args.rviz, args.land_after_check,
                  args.record, args.world_file, args.spawn, args.flight, args.route_dir, args.once)
    elif mode == 'status':
        print(json.dumps(state(), indent=2))
    elif mode == 'stop':
        s = state()
        if s:
            os.kill(s['pid'], signal.SIGTERM)
            for _ in range(100):
                if not state(): break
                time.sleep(0.2)
            if state(): raise RuntimeError('Shutdown timed out; inspect supervisor.log')
        print('Stopped')
    elif mode in ('stack', 'sim'):
        if state(): raise SystemExit('Already running. Use ./run.sh stop first.')
        (ROOT/'logs').mkdir(exist_ok=True)
        with (ROOT/'logs'/'supervisor.log').open('a') as out:
            p = subprocess.Popen([sys.executable, __file__, '_supervise', mode],
                                 stdin=subprocess.DEVNULL, stdout=out, stderr=out,
                                 start_new_session=True)
        time.sleep(3)
        if p.poll() is not None: raise SystemExit('Startup failed; see logs/supervisor.log')
        print('Started in observation-only mode. Use ./run.sh status and ./run.sh check-stack.')
