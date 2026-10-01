#!/usr/bin/env python3
"""Wait for MIMOSA, arm/take off using DRAGON's GT controller, then hold position."""
import argparse
import json
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from flight_slam import Flight, sample


class Hover(Flight):
    def __init__(self, output, land_after_check):
        self.stage = 'initialize'
        self.state = None
        self.cog = None
        self.hover_samples = []
        self.land_after_check = land_after_check
        self.stage_wall = time.monotonic()
        super().__init__(output, node_name='dragon_hover_slam',
                         gt_topic='/dragon/ground_truth', command_topic=None)
        self.create_subscription(UInt8, '/dragon/flight_state', self.on_state, 10)
        self.create_subscription(Odometry, '/dragon/uav/cog/odom', self.on_cog, qos_profile_sensor_data)
        self.commands = {s: self.create_publisher(Empty, '/dragon/teleop_command/'+s, 10)
                         for s in ['start', 'takeoff', 'land']}

    def on_state(self, msg):
        self.state = msg.data

    def on_cog(self, msg):
        self.cog = sample(msg)

    def transition(self, stage, command=None):
        self.stage = stage
        self.stage_wall = time.monotonic()
        self.get_logger().info('Stage: '+stage)
        if command:
            self.commands[command].publish(Empty())

    def fail(self, reason):
        self.failed = reason
        self.get_logger().error(reason)
        self.save()
        self.transition('landing', 'land')

    def control(self):
        if self.done:
            return
        now = time.monotonic()
        if self.stage == 'initialize':
            if (self.state == 0 and self.cog and len(self.est) >= 30 and self.map_points > 1000
                    and self.gt and now-self.last_slam_wall < 2 and now-self.last_gt_wall < 2
                    and self.commands['start'].get_subscription_count()):
                self.origin = np.array(self.cog[1:4])
                self.start = self.gt[-1][0]
                self.map_at_takeoff = self.map_count
                self.transition('arming', 'start')
            elif now-self.start_wall > 300:
                self.failed = 'Initialization timed out: flight state, sensors or MIMOSA unavailable'
                self.save(); self.done = True
                self.get_logger().error(self.failed)
            return
        if now-self.last_clock_wall > 2:
            return  # Pause/resume uses simulated time.
        if self.stage == 'arming':
            if self.state == 2:
                self.transition('takeoff', 'takeoff')
            elif now-self.stage_wall > 60:
                self.fail('Arming timeout')
        elif self.stage == 'takeoff':
            if self.state == 5:
                self.takeoff_confirmed = True
                self.hover_start = self.cog[0]
                self.transition('hover')
            elif now-self.stage_wall > 180:
                self.fail('Takeoff timeout')
        elif self.stage == 'hover':
            elapsed = self.cog[0]-self.hover_start
            if self.state != 5:
                self.fail('Lost HOVER flight state')
                return
            if self.sim_time-self.est[-1][0] > 3:
                self.fail('MIMOSA odometry stale during hover')
                return
            if elapsed > 5:
                self.hover_samples.append(self.cog)
                roll, pitch, _ = Rotation.from_quat(self.cog[4:8]).as_euler('xyz')
                if (abs(self.cog[3]-1.5) > .3 or np.linalg.norm(np.array(self.cog[1:3])-self.origin[:2]) > .4
                        or max(abs(roll),abs(pitch)) > .35):
                    self.fail('Hover pose outside tolerance')
                    return
            if elapsed >= 35:
                self.save()
                self.get_logger().info('30 s stable hover and MIMOSA map saved; holding until Ctrl+C or land command')
                if self.land_after_check:
                    self.transition('landing', 'land')
                else:
                    self.done = True
        elif self.stage == 'landing':
            if self.state == 0:
                self.save(); self.done = True
                self.get_logger().info('Landed; validation saved. Ctrl+C stops the simulation.')
            elif now-self.stage_wall > 180:
                self.failed = self.failed or 'Landing timeout'
                self.save(); self.done = True

    def save(self):
        super().save()
        path = self.output/'flight_slam_result.json'
        report = json.loads(path.read_text())
        report.update(robot='dragon', stage=self.stage, flight_state=self.state,
                      lidar_count=1, gt_reference='FC IMU', control_odometry='estimation.mode=2')
        if self.hover_samples:
            a = np.array(self.hover_samples)
            report.update(hover_duration_s=float(a[-1,0]-a[0,0]),
                          hover_altitude_min_m=float(a[:,3].min()), hover_altitude_max_m=float(a[:,3].max()),
                          hover_xy_max_error_m=float(np.linalg.norm(a[:,1:3]-self.origin[:2],axis=1).max()))
        path.write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--land-after-check', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Hover(args.output, args.land_after_check)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.done:
            node.failed = 'Interrupted before completion'
            node.save()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
