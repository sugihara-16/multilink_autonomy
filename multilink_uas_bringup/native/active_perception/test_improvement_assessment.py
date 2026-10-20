import copy
import hashlib
import json
import tempfile
from pathlib import Path
import unittest
from report_gimbal_improvement import assess,frozen_evidence


class AssessmentTest(unittest.TestCase):
    def setUp(self):
        self.criteria=dict(min_wins=5,per_route_mean_ratio_max=.9,all_pair_mean_ratio_max=.8,max_single_pair_ratio=1.5)
        self.routes=['a','b','c'];self.seeds=[20,21]
        self.rows=[dict(route=r,seed=s,valid=True,active_fixed_position_ratio={'mimosa':.7}) for r in self.routes for s in self.seeds]

    def check(self,rows):return assess(rows,self.criteria,self.routes,self.seeds)

    def test_complete(self):self.assertTrue(self.check(self.rows)['passed'])

    def test_partial_and_invalid(self):
        self.assertFalse(self.check(self.rows[:-1])['passed'])
        self.rows[-1]['valid']=False
        self.assertFalse(self.check(self.rows)['passed'])

    def test_duplicate_or_wrong_seed(self):
        self.rows[1]=copy.deepcopy(self.rows[0])
        self.assertFalse(self.check(self.rows)['passed'])
        self.rows[1]['seed']=22
        self.assertFalse(self.check(self.rows)['passed'])

    def test_large_failure_not_hidden_by_other_wins(self):
        self.rows[0]['active_fixed_position_ratio']['mimosa']=1.6
        for row in self.rows[1:]:row['active_fixed_position_ratio']['mimosa']=.1
        self.assertFalse(self.check(self.rows)['passed'])

    def test_same_code_cannot_hide_changed_render_or_fusion_settings(self):
        frozen={'files':{},'protocol_fields':{'camera_far':200.,'fusion_imu_noise':'legacy'}}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);protocol={'camera_far':200.,'fusion_imu_noise':'legacy'}
            (p/'protocol.json').write_text(json.dumps(protocol))
            self.assertTrue(all(frozen_evidence(p,frozen).values()))
            protocol['camera_far']=None
            (p/'protocol.json').write_text(json.dumps(protocol))
            self.assertFalse(all(frozen_evidence(p,frozen).values()))
            del protocol['camera_far']
            (p/'protocol.json').write_text(json.dumps(protocol))
            self.assertFalse(all(frozen_evidence(p,frozen).values()))

    def test_startup_equivalence_requires_unchanged_generated_inputs(self):
        sha=lambda b:hashlib.sha256(b).hexdigest()
        frozen={'files':{'/root/uas_native/unipilot_gimbal.py':sha(b'atomic writer')}}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'implementation').mkdir();(p/'protocol.json').write_text('{}')
            (p/'implementation/unipilot_gimbal.py').write_bytes(b'previous writer')
            (p/'unipilot_gimbal.sdf').write_bytes(b'same complete model')
            self.assertFalse(all(frozen_evidence(p,frozen).values()))
            frozen['bootstrap_equivalence']=dict(old_source_sha256=sha(b'previous writer'),
                new_source_sha256=sha(b'atomic writer'),flight_output_sha256={'unipilot_gimbal.sdf':sha(b'same complete model')})
            self.assertTrue(all(frozen_evidence(p,frozen).values()))
            (p/'unipilot_gimbal.sdf').write_bytes(b'changed sensor')
            self.assertFalse(all(frozen_evidence(p,frozen).values()))
            # Independent frozen manifests contain no development exception.
            del frozen['bootstrap_equivalence']
            self.assertFalse(all(frozen_evidence(p,frozen).values()))


if __name__=='__main__':unittest.main()
