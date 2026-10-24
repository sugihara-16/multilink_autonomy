"""Verify online launch preparation needs no historical logs."""
import hashlib
import json
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
NATIVE = HERE.parent
INPUTS = NATIVE/'simulation_inputs'


class SimulationInputsTest(unittest.TestCase):
    def test_relocated_input_bytes_and_route_dependencies(self):
        manifest = json.loads((INPUTS/'relocation_manifest.json').read_text())
        for item in manifest['copies']:
            dest = Path(item['destination'])
            self.assertFalse(dest.is_relative_to(NATIVE/'logs'))
            self.assertEqual(hashlib.sha256(dest.read_bytes()).hexdigest(), item['sha256'])
        catalog = json.loads((INPUTS/'zab_catalog.json').read_text())
        self.assertEqual(len(catalog), 14)
        for entry in catalog.values():
            for name, digest in entry['inputs_sha256'].items():
                path = Path(name).resolve()
                self.assertFalse(path.is_relative_to(NATIVE/'logs'))
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        for name in ['paired_flight', 'directional_diagnosis']:
            cmd = json.loads((INPUTS/'templates'/f'{name}.json').read_text())
            for flag in ['--route', '--world', '--planner-config', '--fusion-calibration']:
                path = Path(cmd[cmd.index(flag)+1]).resolve()
                self.assertTrue(path.exists(), path)
                self.assertFalse(path.is_relative_to(NATIVE/'logs'))

    def test_online_preparation_with_historical_logs_forbidden(self):
        original_open = Path.open
        def guarded(path, *args, **kwargs):
            if path.resolve().is_relative_to(NATIVE/'logs'):
                raise AssertionError('Historical log dependency: '+str(path))
            return original_open(path, *args, **kwargs)
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, 'open', guarded):
            commands = [
                ['run_selected_gimbal.py', '--output', tmp+'/selected', '--dry-run'],
                ['unipilot_campaign.py', '--output', tmp+'/paired', '--prepare-only'],
                ['run_lio_failure_comparisons.py', '--output', tmp+'/lio', '--prepare-only'],
                ['run_zab_additional_conditions.py', '--output', tmp+'/additional', '--prepare-only'],
                ['run_view_array_campaign.py', '--output', tmp+'/array', '--prepare-only']]
            for cmd in commands:
                with self.subTest(cmd=cmd[0]), patch.object(sys, 'argv', cmd):
                    runpy.run_path(str(HERE/cmd[0]), run_name='__main__')
            lio = json.loads((Path(tmp)/'lio/campaign.json').read_text())
            self.assertIn('run1_prefix', lio['order'])
            self.assertEqual(lio['runs'], {})
            array = json.loads((Path(tmp)/'array/campaign.json').read_text())
            self.assertEqual(len(array['order']), 6)


if __name__ == '__main__':
    unittest.main()
