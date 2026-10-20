#!/usr/bin/env python3
"""Six error traces as one ROS image, displayed inside RViz's Image panel."""
import argparse
from collections import deque
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class Plot(Node):
    def __init__(self,out):
        super().__init__('odometry_error_plot');self.out=out;self.data={k:deque(maxlen=200000) for k in ('LIO','VIO')};self.first=None
        self.pub=self.create_publisher(Image,'/evaluation/error_plot',2)
        for k in self.data:self.create_subscription(Float64MultiArray,f'/evaluation/{k.lower()}/error',lambda m,k=k:self.receive(k,m),100)
        self.fig,self.axes=plt.subplots(3,2,figsize=(9,7),dpi=100);self.fig.subplots_adjust(left=.1,right=.98,bottom=.08,top=.91,hspace=.40,wspace=.32)
        self.lines={};labels=['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']
        self.axes=list(self.axes[:,0])+list(self.axes[:,1])
        for i,ax in enumerate(self.axes):
            ax.set_ylabel(labels[i]);ax.grid(alpha=.3)
            for k,c in [('LIO','tab:orange'),('VIO','tab:blue')]:self.lines[k,i],=ax.plot([],[],color=c,label=k,lw=1)
        self.axes[0].legend(loc='upper left',ncol=2);self.axes[2].set_xlabel('Simulation time since alignment [s]');self.axes[5].set_xlabel('Simulation time since alignment [s]')
        self.create_timer(1.,self.draw)
    def receive(self,k,m):
        if len(m.data)!=7:return
        self.data[k].append(m.data)
        if self.first is None:self.first=m.data[0]
    def draw(self):
        title='LIO / VIO error vs time-matched ground truth'
        if self.first is None:title+=' — waiting for odometry'
        self.fig.suptitle(title+'\nFixed initial alignment; position in world, rotation relative to truth',fontsize=11)
        latest=max((v[-1][0] for v in self.data.values() if v),default=0)
        for k,v in self.data.items():
            if not v:continue
            a=np.array(v);a=a[a[:,0]>=latest-180] # CSV retains the complete history.
            for i in range(6):self.lines[k,i].set_data(a[:,0]-self.first,a[:,i+1])
        for ax in self.axes:
            ax.relim();ax.autoscale_view()
        self.fig.canvas.draw();rgba=np.asarray(self.fig.canvas.buffer_rgba());rgb=rgba[:,:,:3].copy()
        m=Image();m.header.stamp=self.get_clock().now().to_msg();m.height,m.width=rgb.shape[:2];m.encoding='rgb8';m.step=m.width*3;m.data=rgb.tobytes();self.pub.publish(m)
        self.fig.savefig(self.out/'error_plot.png')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();rclpy.init();n=Plot(args.output)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();plt.close('all')
