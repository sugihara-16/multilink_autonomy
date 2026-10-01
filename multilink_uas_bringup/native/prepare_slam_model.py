"""Derive a simulation-only sensor model without modifying the upstream model."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def prepare_model(source, destination):
    source = Path(source)
    text = source.read_text()
    # Enable IMU only, retaining upstream noise parameters.
    imu = re.search(r'<sensor name="imu" type="imu">.*?</sensor>', text, re.S).group(0)
    root = ET.fromstring(text)
    model = root.find('model')
    model.find("link[@name='base_link']").append(ET.fromstring(imu))
    # The velocity controller alone owns the motors for this flight mode.
    for plugin in list(model.findall('plugin')):
        if 'AccelerationControl' in plugin.get('name', ''):
            model.remove(plugin)
        elif 'MulticopterVelocityControl' in plugin.get('name', ''):
            # Increase vertical feedback for altitude tracking in this flight demo.
            plugin.find('velocityGain').text = '4 4 10'
    for uri in root.iter('uri'):
        if uri.text.startswith('meshes/'):
            uri.text = str(source.parent / uri.text)
    ET.ElementTree(root).write(destination, encoding='unicode', xml_declaration=True)
