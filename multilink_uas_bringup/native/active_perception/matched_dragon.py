"""Match the established UniPilot experiment without modifying frozen inputs."""
import hashlib
import json
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
import shlex
from actuator_reachability import parameters
from view_preparation import replace_once

HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]

def runtime_environment(name):
    """Reuse the UniPilot transport retries, renderer and thread limits."""
    rendering=WS/'build/gz-rendering-speed-install/lib'
    required=[rendering/'gz-rendering-8/engine-plugins/libgz-rendering-ogre2.so']
    script=f'export OMP_NUM_THREADS=2 OMP_WAIT_POLICY=PASSIVE\nexport GZ_RENDERING_PLUGIN_PATH={rendering}/gz-rendering-8/engine-plugins\nexport LD_LIBRARY_PATH={rendering}:${{LD_LIBRARY_PATH:-}}\n'
    if name=='simulation':
        retry=WS/'build/zmq-retry/libzmq_eintr_retry.so'
        stop=WS/'build/gz-stop/lib/libgz-sim8.so.8.15.0'
        guard=WS/'build/no-crash-dump/libno_crash_dump.so'
        required.extend([retry,stop,guard])
        retained=Path('/usr/lib/wsl/lib/libd3d12core.so')
        preload=':'.join(map(str,[guard,retry]+([retained] if retained.exists() else [])))
        script+=f'export LD_PRELOAD={shlex.quote(preload)}:${{LD_PRELOAD:-}}\nexport LD_LIBRARY_PATH={stop.parent}:${{LD_LIBRARY_PATH:-}}\nexport GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${{GZ_SIM_SYSTEM_PLUGIN_PATH:-}}\n'
    if name=='mimosa':
        limit=WS/'build/mimosa-speed/libtbb_limit.so';required.append(limit)
        script+=f'export UAS_TBB_THREADS=1 LD_PRELOAD={limit}:${{LD_PRELOAD:-}}\n'
    for path in required:
        if not path.exists():raise FileNotFoundError(path)
    return script

def prepare_world(out,world):
    root=ET.parse(world).getroot();w=root.find('world')
    policy=ET.SubElement(w,'gz:policies',{'xmlns:gz':'https://gazebosim.org'})
    ET.SubElement(policy,'parallel_postupdates').text='false'
    plugin=HERE.parent.parents[3]/'build/gz-batch-speed/libgz-sim8-sensors-system.so'
    if not plugin.exists():raise FileNotFoundError(plugin)
    w.find("plugin[@name='gz::sim::systems::Sensors']").set('filename',str(plugin))
    path=out/'world_serial.sdf';ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
    return path

def prepare_runtime(out,world,mechanism_comparison=False):
    cfg=yaml.safe_load((out/'planner.yaml').read_text())
    cfg.update(yaml.safe_load((HERE/'motion_view_v10.yaml').read_text()))
    cfg.update(parameters(out/'unipilot_gimbal.sdf',world))
    fusion=yaml.safe_load((out/'mimosa.yaml').read_text())
    cfg.update(view_information_mode='lidar_conditioned',
               fusion_velocity_covariance_scale=fusion['odometry']['manager']['velocity_covariance_scale'],
               fusion_velocity_period_s=fusion['odometry']['manager']['velocity_period_s'])
    if mechanism_comparison:cfg.update(body_joint_speed_deg_s=7.,body_joint_acceleration_deg_s2=14.)
    (out/'planner.yaml').write_text(yaml.safe_dump(cfg))
    (out/'route').mkdir()
    # Same catch-up scheduling as the held-out UniPilot experiments. The
    # planner emits a completion even when holding its current viewing angle.
    source=replace_once((HERE/'pace_simulation.py').read_text(),"'/active/status'","'/active/scheduling_complete'")
    source=replace_once(source,'    log.close()', '    for sub in subs:sub.unregister()\n    with lock:log.close()')
    (out/'matched_pacer.py').write_text(source)
    (out/'matching.json').write_text(json.dumps(dict(
        reference='information-holdout-20261026 / layout24301 lidar_conditioned',
        planner='ablation_planner_node.py, lidar_conditioned; same v10 configuration',
        preparation='8 s settle, stationary initialization, 12 s approach, two 26 s excitation cycles; hold view in cycle 2; no extra sweep',
        actuation='Direct Gazebo position commands; same per-axis speed and planning constraints as UniPilot',
        route='Same segment_value trapezoid: 90m at 0.5m/s, 2s linear velocity ramps; actual body controller differs',
        simulation='Same batched sensor plugin and serial post-update policy as UniPilot; physics dt unchanged',
        transport='Same interrupted-send retry, inline renderer, safe shutdown and one-thread TBB fusion runtime as UniPilot',
        residual_differences=['DRAGON physical dynamics and link4 attachment',
          'Camera centre FC (0.8,0,-0.025) vs UniPilot (0.18,0,-0.025) to avoid self occlusion',
          'LiDAR centre FC (-0.15,0,0.30) vs UniPilot (-0.15,0,0.05); original origin was inside DRAGON rotor module, actual mounting calibration updated',
          'Independent sensor realizations, even with the same requested seed',
          'Only main camera rendered; no unused fixed-camera diagnostic estimators'],
        truth_usage='flight control and evaluation only',
        estimator='Exact UniPilot ROVIO binary; measured extrinsics; existing 10 micrometre translation guard remains enabled',
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}
    ),indent=2)+'\n')

    if mechanism_comparison:
        path=out/'matching.json';record=json.loads(path.read_text())
        record.update(reference='Symmetric DRAGON joint/gimbal comparison',
            planner='Shared articulated_planner.py map and correlated velocity scoring; different reachable candidates',
            preparation='8 s settle, initialization gate, 12 s approach, two 26 s excitation cycles, no camera or body-joint sweep',
            actuation='Body joints commanded at 7 deg/s, acceleration 14 deg/s2, physical audit limit unchanged at 12 deg/s, or optical-centre gimbal at 14 deg/s; actual measured joints used',
            estimator='Same full SE3 ROVIO binary for both; measured camera translation and orientation at each image stamp',
            residual_differences=['Six body axes vs two gimbal axes, attainable camera poses and path constraints',
                'Resulting body and sensor motions, including camera translation for the joint arm',
                'Independent physical sensor noise series even with equal requested seed'])
        path.write_text(json.dumps(record,indent=2)+'\n')
