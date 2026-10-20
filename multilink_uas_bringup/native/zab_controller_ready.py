#!/usr/bin/env python3
"""Recover controller activation after large-world GPU initialization; never arm."""
import time,argparse,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);args=p.parse_args()
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from controller_manager_msgs.srv import ListControllers,SwitchController
rclpy.init();n=Node('zab_controller_ready');clock=[0.]
n.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
l=n.create_client(ListControllers,'/dragon/controller_manager/list_controllers');s=n.create_client(SwitchController,'/dragon/controller_manager/switch_controller')
required={'joint_state_broadcaster','attitude_controller','joint_group_position_controller','gimbal_position_controller'}
end=time.monotonic()+180
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.2)
 if clock[0]<1 or not l.service_is_ready() or not s.service_is_ready():continue
 f=l.call_async(ListControllers.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if not f.done() or f.result() is None:continue
 states={c.name:c.state for c in f.result().controller}
 if not required<=states.keys():
  for name in sorted(required-states.keys()):
   subprocess.run(['ros2','run','controller_manager','spawner',name,'--controller-manager','/dragon/controller_manager',
                   '--controller-manager-timeout','60','--switch-timeout','60','--param-file',str(args.output/'Simulation.yaml')],timeout=100)
  continue
 if all(states[k]=='active' for k in required):
  n.get_logger().info('All four DRAGON controllers active; robot remains disarmed');break
 inactive=[k for k in required if states[k]=='inactive']
 if inactive:
  q=SwitchController.Request();q.activate_controllers=inactive;q.strictness=1;q.timeout.sec=60
  f=s.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=65)
else:raise RuntimeError('DRAGON controllers did not become ready within 180 seconds')
n.destroy_node();rclpy.shutdown()
