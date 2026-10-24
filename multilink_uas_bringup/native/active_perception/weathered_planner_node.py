#!/usr/bin/env python3
"""Unchanged guarded planner, plus a separate scheduling-completion signal."""
from pathlib import Path
import core
from motion_view_planner import Planner
from view_preparation import planner_source,replace_once
core.Planner=Planner
path=Path(__file__).with_name('planner_node.py')
source=planner_source(path.read_text())
source=replace_once(source,'from retinal_tracking_view import RetinalTrackingView as Fixation','from committed_tracking_view import CommittedTrackingView as Fixation')
source=replace_once(source,"        self.status=rospy.Publisher('/active/status',String,queue_size=1)","        self.status=rospy.Publisher('/active/status',String,queue_size=1)\n        self.scheduling=rospy.Publisher('/active/scheduling_complete',String,queue_size=1)")
source=replace_once(source,"if __name__=='__main__':", "from scheduling_completion import completed_attempt\ndef notify_completion(self):\n    self.scheduling.publish(String(data=json.dumps(dict(stamp_ns=rospy.Time.now().to_nsec()))))\nActive.tick=completed_attempt(Active.tick,notify_completion)\n\nif __name__=='__main__':")
exec(compile(source,str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
