import ast,types,unittest
from pathlib import Path
import numpy as np
import initialization_motion
from view_preparation import route_source,planner_source
H=Path(__file__).resolve().parent
class PreparationTest(unittest.TestCase):
 def test_both_cycles_use_identical_existing_motion(self):
  tree=ast.parse(route_source((H/'unipilot_route.py').read_text()));cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Flight');fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='control')
  env={'initialization_motion':initialization_motion,'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'control','exec'),env)
  class Flight:
   def __init__(self):self.stage='excitation';self.stage_time=0.;self.last=None;self.failure=None;self.target=np.array([1.,2.,3.]);self.segments=[{'yaw0':.1}];self.tracking=0.;self.clock=0.;self.gt=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0,nanosec=0)))
   def now(self):return self.clock
   def command(self,p,v,y,w):self.output=(p,v,y,w)
   def transition(self,s):self.stage=s;self.stage_time=self.clock
   def status(self):pass
  f=Flight();first=[];second=[]
  for cycle in range(2):
   for t in [0.,1.,2.,5.,10.,18.,23.,25.]:
    f.clock=cycle*26+t;f.gt.header.stamp.sec=int(f.clock);env['control'](f)
    expected=initialization_motion.reference(t);np.testing.assert_allclose(f.output[0],f.target+expected[0]);np.testing.assert_allclose(f.output[1],expected[1]);(first if cycle==0 else second).append(f.output)
   f.clock=(cycle+1)*26;f.gt.header.stamp.sec=int(f.clock);env['control'](f);self.assertEqual(f.stage,'excitation_hold' if cycle==0 else 'tracking')
  for a,b in zip(first,second):np.testing.assert_array_equal(a[0],b[0]);np.testing.assert_array_equal(a[1],b[1])
 def test_hold_uses_encoder_without_modifying_fixed_baseline(self):
  tree=ast.parse(planner_source((H/'planner_node.py').read_text()));node=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="self.args.mode == 'fixed' or waiting")
  fn=ast.parse('def select(self,cur,stage,waiting):\n pass\n').body[0];fn.body=[node,ast.parse('return target,report').body[0]];ast.fix_missing_locations(fn);env={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'select','exec'),env)
  obj=types.SimpleNamespace(args=types.SimpleNamespace(mode='active'),cfg={'hold_active_view_in_second_cycle':True});cur=np.array([.4,-.6]);target,report=env['select'](obj,cur,'excitation_hold',False);np.testing.assert_array_equal(target,cur);self.assertIsNot(target,cur);self.assertEqual(report['reason'],'hold_view_for_common_preparation')
  obj.args.mode='fixed';target,report=env['select'](obj,cur,'excitation_hold',False);np.testing.assert_array_equal(target,[0.,0.]);self.assertEqual(report['reason'],'fixed_baseline')
 def test_missing_anchor_fails_closed(self):
  with self.assertRaises(ValueError):route_source('')
  with self.assertRaises(ValueError):planner_source('')
if __name__=='__main__':unittest.main()
