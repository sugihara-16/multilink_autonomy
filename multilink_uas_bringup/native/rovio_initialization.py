"""Sensor-derived initialization settings for the recorded DRAGON simulation.

This changes the uncertainty of the initial accelerometer bias, not its mean.
It consumes the sensor specification only; never a measured ground-truth pose,
trajectory error, or fitted bias from a target recording.
"""
import re
import xml.etree.ElementTree as ET


def match_accelerometer_bias_prior(config, urdf_path, sensor_name='spinal_imu'):
    root = ET.parse(urdf_path).getroot()
    sensor = root.find(f".//sensor[@name='{sensor_name}']/imu/linear_acceleration")
    if sensor is None:
        raise ValueError('Recorded spinal_imu accelerometer specification is missing')
    variances = []
    for axis in 'xyz':
        noise = sensor.find(axis + '/noise')
        if noise is None or noise.get('type') != 'gaussian':
            raise ValueError('Expected explicit Gaussian accelerometer bias specification')
        mean = float(noise.findtext('bias_mean'))
        std = float(noise.findtext('bias_stddev'))
        # Conservative zero-mean prior: second moment includes the nominal bias.
        # It remains valid without assuming the sign of this recording's bias.
        variances.append(mean * mean + std * std)
    match = re.search(r'(Init\s*\{.*?Covariance\s*\{)([^}]*)(\})', config, re.S)
    if match is None:
        raise ValueError('ROVIO Init.Covariance block missing')
    block = match[2]
    for i, variance in enumerate(variances):
        block, count = re.subn(rf'(acb_{i}\s+)[^;\s]+;',
                               lambda m: m[1] + format(variance, '.12g') + ';', block)
        if count != 1:
            raise ValueError(f'Expected exactly one initial accelerometer bias axis {i}')
    return config[:match.start(2)] + block + config[match.end(2):], variances
