"""Geometry and isolation checks before the simultaneous live experiment."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import warnings
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from fixed_view_array import VIEWS, OPTICAL, prepare_array
from run_view_array import effective_sources
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from unipilot_gimbal import prepare


class ViewArrayTest(unittest.TestCase):
    def test_model_angles_and_unchanged_physics(self):
        reference = Path(__file__).resolve().parent.parent/'simulation_inputs/model_source.json'
        source = json.loads(reference.read_text())['model_source']
        with tempfile.TemporaryDirectory() as tmp, warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            out = Path(tmp)
            prepare_array(prepare, source, out, paired=True, camera_far=1000.)
            original = ET.parse(out/'array_model_base/unipilot_gimbal.sdf')
            actual = ET.parse(out/'unipilot_gimbal.sdf')
            body = actual.getroot().find("model/link[@name='base_link']")
            original_body = original.getroot().find("model/link[@name='base_link']")
            template = original_body.find("sensor[@name='fixed_rgb_cam']")
            expected_forward = dict(
                yaw_m50=[np.cos(np.deg2rad(50)), -np.sin(np.deg2rad(50)), 0],
                yaw_p50=[np.cos(np.deg2rad(50)), np.sin(np.deg2rad(50)), 0],
                pitch_p90=[0, 0, -1], pitch_m90=[0, 0, 1])
            for name in VIEWS:
                sensor = body.find(f"sensor[@name='{name}_rgb_cam']")
                pose = np.fromstring(sensor.findtext('pose'), sep=' ')
                rotation = Rotation.from_euler('xyz', pose[3:]).as_matrix()
                np.testing.assert_allclose(rotation@OPTICAL@np.array([0, 0, 1]),
                                           expected_forward[name], atol=1e-12)
                np.testing.assert_allclose(pose[:3], [.18, 0, -.025], atol=1e-12)
                self.assertEqual(ET.tostring(sensor.find('camera')),
                                 ET.tostring(template.find('camera')))
                body.remove(sensor)
            self.assertEqual(ET.tostring(actual.getroot()), ET.tostring(original.getroot()))

    def test_owner_extension_compiles_and_drains_all_views(self):
        owner, pacer, diff = effective_sources()
        compile(owner, 'array_owner', 'exec')
        compile(pacer, 'array_pacer', 'exec')
        self.assertIn("required['array_vio_'+name]", owner)
        self.assertIn("range(len(args.pacing_extra_lidar))", owner)
        self.assertIn('start_views(out', diff)
        self.assertIn("key='array_vio_'+view", pacer)


if __name__ == '__main__':
    unittest.main()
