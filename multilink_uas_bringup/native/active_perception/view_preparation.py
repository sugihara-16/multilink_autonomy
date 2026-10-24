"""Explicit common preparation protocol for a diagnostic candidate.

Both fixed and active estimators receive the same additional body motion.
Only the active gimbal holds its measured view during the second preparation.
No truth or controller reference is passed to the view planner.
"""
def replace_once(source,old,new):
    if source.count(old)!=1:raise ValueError('Frozen source anchor changed: '+old[:90])
    return source.replace(old,new,1)

def route_source(source):
    source=replace_once(source,"elif self.stage=='excitation':","elif self.stage in ('excitation','excitation_hold'):")
    source=replace_once(source,"elif elapsed>=initialization_motion.DURATION and self.tracking<.05:\n                self.transition('tracking')", "elif elapsed>=initialization_motion.DURATION and self.tracking<.05:\n                self.transition('excitation_hold' if self.stage=='excitation' else 'tracking')")
    return source

def planner_source(source):
    source=replace_once(source,"waiting=False\n", "waiting=False;stage=None\n")
    source=replace_once(source,"try:waiting=json.loads((self.args.output/'route_status.json').read_text())['stage'] not in ('tracking','finish_hold','landing','complete')", "try:\n                    stage=json.loads((self.args.output/'route_status.json').read_text())['stage']\n                    waiting=stage not in ('approach','excitation','excitation_hold','tracking','finish_hold','landing','complete')")
    source=replace_once(source,"if self.args.mode=='fixed' or waiting:target=np.zeros(2);report=dict(reason='fixed_baseline')", "if self.args.mode=='fixed' or waiting:target=np.zeros(2);report=dict(reason='fixed_baseline')\n            elif stage=='excitation_hold' and self.cfg.get('hold_active_view_in_second_cycle',True):target=cur.copy();report=dict(reason='hold_view_for_common_preparation')")
    return source
