#!/usr/bin/env python3
"""Small ROS launch lifecycle test; no Gazebo, sensors or flight are started."""
import argparse,json,os,signal,subprocess,tempfile,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    reports=[]
    with tempfile.TemporaryDirectory(prefix='gimbal-launch-stop-') as temp:
        directory=Path(temp);launch=directory/'stop.launch.py'
        launch.write_text('''from launch import LaunchDescription
from launch.actions import ExecuteProcess,RegisterEventHandler,Shutdown
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    server=ExecuteProcess(cmd=['python3','-c','import time; time.sleep(.4)'],output='screen',name='server')
    peer=ExecuteProcess(cmd=['python3','-c','import signal,sys,time; signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); time.sleep(60)'],output='screen',name='peer')
    return LaunchDescription([server,peer,RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[Shutdown(reason="server stopped")]))])
''')
        for repeat in range(5):
            log=directory/f'{repeat}.log'
            with log.open('w') as f:
                proc=subprocess.Popen(['ros2','launch',str(launch)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            start=time.monotonic()
            try:code=proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait();raise RuntimeError(log.read_text())
            text=log.read_text();clean=text.count('process has finished cleanly')
            reports.append(dict(repeat=repeat,parent_exit=code,clean_children=clean,wall_s=time.monotonic()-start))
            if code or clean!=2:raise RuntimeError(text)
    a.output.write_text(json.dumps(dict(scope='Automatic launch Shutdown after server completion, without a redundant parent signal',runs=reports),indent=2)+'\n')
    print('Five automatic-shutdown launch parents and ten child processes: all exit 0')


if __name__=='__main__':main()
