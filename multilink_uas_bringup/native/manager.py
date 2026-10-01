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

def supervise(mode, headless='true', rviz='false', land_after_check='false'):
    # One owner for launch and the legacy run.sh entry points alike.
    lock = (ROOT / '.supervisor.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('UAS is already running; stop the existing launch first')
    children = []
    logdir = ROOT / 'logs' / (time.strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}')
    logdir.mkdir(parents=True)
    current = ROOT / 'logs' / 'latest'
    current.unlink(missing_ok=True)
    current.symlink_to(logdir.name)
    status = dict(pid=os.getpid(), starttime=identity(os.getpid()), mode=mode,
                  logs=str(logdir), components={})
    STATE.write_text(json.dumps(status, indent=2))
    print(f'UAS {mode}: logs at {logdir}', flush=True)
    def shutdown(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    def launch(name, command, ros1=False):
        envfile = ROOT / ('env_ros1.sh' if ros1 else ('env_dragon.sh' if mode == 'dragon_slam' else 'env.sh'))
        # ROS1 core must be usable before the planner overlay is available too.
        command = f'source {shlex.quote(str(envfile))} && exec {command}'
        out = (logdir / f'{name}.log').open('w')
        p = subprocess.Popen(['bash', '-c', command], cwd=ROOT,
                             stdout=out, stderr=subprocess.STDOUT, start_new_session=True, env=CLEAN)
        out.close()
        children.append(p)
        status['components'][name] = p.pid
        STATE.write_text(json.dumps(status, indent=2))
    try:
        if mode in ('stack', 'flight_slam', 'dragon_slam', 'vio_slam'):
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
            bridge_config = {'flight_slam': 'bridge_slam.yaml', 'dragon_slam': 'bridge_dragon.yaml', 'vio_slam': 'bridge_vio.yaml'}.get(mode, 'bridge_params.yaml')
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
        if mode == 'dragon_slam':
            (logdir/'mimosa').mkdir()
            launch('slam_inputs', f'python3 {ROOT}/slam_inputs.py _imu_topic:=/dragon/imu/raw _points_topic:=/dragon/lidar/points', True)
            launch('mimosa', f'bash -c "source {SLAM_WS}/devel/setup.bash && exec roslaunch {ROOT}/mimosa_unipilot.launch config:={ROOT}/mimosa_dragon.yaml logs:={logdir}/mimosa"', True)
            launch('sim', f'ros2 launch {ROOT}/dragon_sim.launch.py headless:={headless} output:={logdir}')
            launch('hover', f'python3 {ROOT}/dragon_hover.py --output {logdir}' + (' --land-after-check' if land_after_check == 'true' else ''))
        else:
            launch('sim', f'ros2 launch {ROOT}/sim_wsl.launch.xml headless:={headless}{model_arg}')
        if mode == 'flight_slam':
            launch('flight', f'python3 {ROOT}/flight_slam.py --output {logdir}')
        if rviz == 'true':
            rviz_config = 'slam.rviz' if mode in ('flight_slam', 'dragon_slam', 'vio_slam') else 'unipilot.rviz'
            if mode == 'vio_slam': rviz_config = 'vio.rviz'
            render_env = 'env LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 ' if mode in ('dragon_slam', 'vio_slam') else ''
            launch('rviz', f'{render_env}rviz2 -d {ROOT}/{rviz_config}' + (' --ros-args -p use_sim_time:=true' if mode == 'vio_slam' else ''))
        if mode == 'stack':
            launch('pc_selector', 'ros2 launch robot_bringup cbf_pc_selector_sim.launch.py cfg:=cbf_pc_selector_sim_unipilot.yaml')
            launch('cbf', shlex.join(['ros2', 'run', 'composite_cbf', 'composite_cbf_node', '--ros-args', '-r', '__node:=composite_cbf', '--params-file', str(UAS/'robot_bringup/config/ros2/cbf_sim_unipilot.yaml'), '-p', 'use_sim_time:=true', '-r', '~/obstacles:=/cbf_pc_selector/output_pc', '-r', '~/odom:=/rmf_unipilot/odom', '-r', '~/cmd_in:=/sdf_nmpc/cmd/acc', '-r', '~/safe_cmd_twist:=/validation/cbf_safe_cmd']))
            launch('rl', f'bash -c "source /root/.venvs/uas/bin/activate && cd {UAS}/ws_rl/src/rl_nav/scripts && exec python3 lidar_nav_acc_ros2_sim_node.py --env=lidar_navigation_task --experiment=magpie_accel_policy4"')
        while True:
            for p in children:
                if p.poll() is not None:
                    raise RuntimeError(f'component {p.pid} exited: {p.returncode}; see {logdir}')
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
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
        parser.add_argument('mode', choices=['sim', 'stack', 'flight_slam', 'dragon_slam', 'vio_slam'])
        parser.add_argument('--headless', choices=['true', 'false'], default='true')
        parser.add_argument('--rviz', choices=['true', 'false'], default='false')
        parser.add_argument('--land-after-check', choices=['true', 'false'], default='false')
        args = parser.parse_args(sys.argv[2:])
        supervise(args.mode, args.headless, args.rviz, args.land_after_check)
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
