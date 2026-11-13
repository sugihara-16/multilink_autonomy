"""Fetch pinned simulator/estimator sources into the isolated experiment runtime."""
import json, os, subprocess, re
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import yaml
NATIVE=Path('/root/uas_native'); WS=Path('/root/multilink_autonomy_ws'); UAS=Path('/root/unified_autonomy_stack')
env=dict(os.environ,GIT_LFS_SKIP_SMUDGE='1')
def clone(spec):
    path,url,rev=spec;path=Path(path)
    url=url.replace('git@github.com:','https://github.com/')
    if not (path/'.git').exists():
        path.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run(['git','clone','--no-checkout',url,str(path)],env=env,check=True)
    target=rev if re.fullmatch(r'[0-9a-f]{40}',rev) else 'origin/'+rev
    subprocess.run(['git','-C',str(path),'fetch','origin',rev],env=env,check=True)
    subprocess.run(['git','-C',str(path),'checkout','--detach',target],env=env,check=True)
    return {'path':str(path),'url':url,'revision':subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()}
base=clone((UAS,'https://github.com/ntnu-arl/unified_autonomy_stack.git','cb78bf7f57d074fc0999dd23bb1cfa1b1754d44d'))
items=[('/root/ros2/aerial_robot_base_ws/src/aerial_robot_base','https://github.com/sugihara-16/aerial_robot_base.git','37022750b6c08d879f0162f0c9a23a393114c061')]
for manifest,root,only in [('ws_sim_exact.repos',UAS/'workspaces/ws_sim',{'src/rmf_gz','src/subt_cave_sim'}),('ws_mimosa_exact.repos',WS/'slam_ros1_ws',None),('robot_bringup_exact.repos',UAS/'workspaces',None)]:
    for name,repo in yaml.safe_load((UAS/'repos'/manifest).read_text())['repositories'].items():
        if only is None or name in only:items.append((root/name,repo['url'],repo['version']))
for name,repo in json.loads((NATIVE/'rovio-source-commits.json').read_text()).items():
    if name!='rovio/lightweight_filtering':items.append((WS/'slam_ros1_ws/src'/name,repo['url'],repo['commit']))
items.extend([
('/root/ros2/aerial_robot_base_ws/src/kalman_filter','https://github.com/ut-dragon-lab/kalman_filter.git','ac259ae0f2925bae99bce741a4d48f436f0901c8'),
('/root/ros2/aerial_robot_base_ws/src/aerial_robot_nerve','https://github.com/ut-dragon-lab/aerial_robot_nerve.git','6ab39c07ad9349c14ef5c50fd89b30fd5b18ee7a'),
('/root/ros2/aerial_robot_base_ws/src/ros2_numpy','https://github.com/Box-Robotics/ros2_numpy.git','d665add22e6a9070e88d9514bbf0dcbcb122a3dc'),
('/root/ros2/aerial_robot_base_ws/src/gz_ros2_control','https://github.com/ros-controls/gz_ros2_control.git','c88a5fd9170af120c263c1201f0744a40f93d673'),
('/root/unified_autonomy_stack/workspaces/ws_ros1_bridge/src/ros1_bridge','https://github.com/ros2/ros1_bridge.git','174d7aad47e9ae9b4c986f253c93f1eaf4620adc')])
with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(clone,items))
results.insert(0,base)
lwf=json.loads((NATIVE/'rovio-source-commits.json').read_text())['rovio/lightweight_filtering']
results.append(clone((WS/'slam_ros1_ws/src/rovio/lightweight_filtering',lwf['url'],lwf['commit'])))
Path('/root/setup-logs/sources.json').write_text(json.dumps(results,indent=2))
