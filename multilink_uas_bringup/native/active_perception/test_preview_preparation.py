import ast,json,types,unittest
from pathlib import Path
import numpy as np
import initialization_motion,preview_motion
from preview_preparation import route_source,planner_source

H=Path(__file__).resolve().parent


class PreviewTests(unittest.TestCase):
    def test_smooth_speed_bounded_profile(self):
        for length in [.2,6.5,15.]:
            duration=preview_motion.profile(0,length)[2]
            ts=np.linspace(0,duration,1001)
            values=np.array([preview_motion.profile(t,length)[:2] for t in ts])
            self.assertGreaterEqual(values[:,1].min(),0.)
            self.assertLessEqual(values[:,1].max(),.5+1e-12)
            np.testing.assert_allclose(values[[0,-1]],[[0,0],[length,0]],atol=1e-12)
            for t in np.linspace(.01,duration-.01,31):
                numeric=(preview_motion.profile(t+1e-5,length)[0]-preview_motion.profile(t-1e-5,length)[0])/2e-5
                self.assertAlmostEqual(numeric,preview_motion.profile(t,length)[1],places=8)

    def test_existing_routes_are_retraced_without_shortcut(self):
        paths=[H.parent/'simulation_inputs/routes'/name/'segments.json' for name in ['run1_tail_a1','run1_tail_b1']]
        paths.append(Path('/root/multilink_autonomy_ws/assets/futatsugoya/routes/interior/segments.json'))
        for path in paths:
            s=json.loads(path.read_text())['segments'][0]
            duration=preview_motion.reference(s,0)[2]
            np.testing.assert_allclose(preview_motion.reference(s,0)[0],s['start'])
            np.testing.assert_allclose(preview_motion.reference(s,duration,True)[0],s['start'])
            for t in np.linspace(0,duration,101):
                p,v,_=preview_motion.reference(s,t);q,w,_=preview_motion.reference(s,duration-t,True)
                np.testing.assert_allclose(p,q,atol=1e-10);np.testing.assert_allclose(v,-w,atol=1e-10)
                self.assertLessEqual(np.linalg.norm(v),.5+1e-9)

    def test_real_control_stage_order_keeps_evaluation_clock_zero(self):
        tree=ast.parse(route_source((H/'unipilot_route.py').read_text()))
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Flight')
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='control')
        env={'np':np,'initialization_motion':initialization_motion,'preview_motion':preview_motion}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'control','exec'),env)
        segment=dict(kind='polyline',points=[[0,0,0],[5,0,0]],arc=[0,5],yaw0=0.,yaw1=0.)
        f=types.SimpleNamespace(stage='excitation',stage_time=0.,last=None,failure=None,target=np.zeros(3),segments=[segment],plan={},tracking=0.,clock=0.,route_time=0.,gt=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0,nanosec=0))))
        f.now=lambda:f.clock;f.status=lambda:None
        f.command=lambda p,v,y,w:None
        def transition(stage):f.stage=stage;f.stage_time=f.clock
        f.transition=transition
        duration=preview_motion.reference(segment,0)[2]
        for clock,stage in [(26,'preview_out'),(26+duration,'preview_return'),(26+2*duration,'excitation_hold'),(52+2*duration,'tracking')]:
            f.clock=clock;f.gt.header.stamp.sec=int(clock);env['control'](f)
            self.assertEqual(f.stage,stage);self.assertEqual(f.route_time,0.)

    def test_hold_on_return_and_preparation_preserves_fixed(self):
        tree=ast.parse(planner_source((H/'planner_node.py').read_text()))
        node=next(x for x in ast.walk(tree) if isinstance(x,ast.If) and ast.unparse(x.test)=="self.args.mode == 'fixed' or waiting")
        fn=ast.parse('def select(self,cur,stage,waiting):\n pass\n').body[0]
        fn.body=[node,ast.parse('return target,report').body[0]];ast.fix_missing_locations(fn)
        env={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'select','exec'),env)
        obj=types.SimpleNamespace(args=types.SimpleNamespace(mode='active'),cfg={'hold_active_view_in_second_cycle':True});cur=np.array([.3,-1.])
        for stage in ['preview_return','excitation_hold']:
            target,_=env['select'](obj,cur,stage,False);np.testing.assert_array_equal(target,cur)
        obj.args.mode='fixed';target,_=env['select'](obj,cur,'preview_return',False);np.testing.assert_array_equal(target,[0.,0.])

    def test_generated_owner_and_sources_compile(self):
        from run_preview_view_array import preview_sources
        source,pacer,_=preview_sources()
        compile(source,'owner','exec');compile(pacer,'pacer','exec')
        compile(route_source((H/'unipilot_route.py').read_text()),'route','exec')
        compile(planner_source((H/'planner_node.py').read_text()),'planner','exec')
        with self.assertRaises(ValueError):route_source('')


if __name__=='__main__':unittest.main()
