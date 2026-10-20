"""MIMOSA bias evolution comes from the actual generated sensor, not errors."""
import math
import xml.etree.ElementTree as ET


def parameters(path):
    imu=ET.parse(path).find(".//sensor[@name='imu']")
    if imu is None:raise ValueError('Missing IMU sensor')
    tau=[];driving=[]
    for kind in ['linear_acceleration','angular_velocity']:
        for axis in 'xyz':
            noise=imu.find(f'imu/{kind}/{axis}/noise')
            if noise is None or noise.get('type')!='gaussian' or float(noise.findtext('mean','0'))!=0:
                raise ValueError('Sensor model requires zero-mean Gaussian white noise')
            t=float(noise.findtext('dynamic_bias_correlation_time','0'));q=float(noise.findtext('dynamic_bias_stddev','0'))
            if not math.isfinite(t) or not math.isfinite(q) or min(t,q)<=0:raise ValueError('Six positive decay times and driving noise amplitudes required')
            tau.append(t);driving.append(q)
    return dict(sensor_bias_model=True,bias_tau_s=tau,bias_driving_sigma=driving)
