"""Acquire a representative moving view, then prepare with that view held."""
from view_preparation import replace_once, route_source as held_route, planner_source as held_planner


def route_source(source):
    source = held_route(source)
    source = replace_once(source, 'import initialization_motion', 'import initialization_motion\nimport preview_motion')
    source = replace_once(source,
        "self.transition('excitation_hold' if self.stage=='excitation' else 'tracking')",
        "self.transition('preview_out' if self.stage=='excitation' else 'tracking')")
    source = replace_once(source, "        elif self.stage=='tracking':", """        elif self.stage in ('preview_out','preview_return'):
            elapsed=now-self.stage_time
            p,v,duration=preview_motion.reference(self.segments[0],elapsed,self.stage=='preview_return')
            self.command(p,v,self.segments[0]['yaw0'],0.)
            if self.tracking>self.plan.get('tracking_stop_m',.8):
                self.failure=f'Preview tracking error {self.tracking:.3f} m';self.status()
            elif elapsed>=duration and self.tracking<.05:
                self.transition('preview_return' if self.stage=='preview_out' else 'excitation_hold')
        elif self.stage=='tracking':""")
    return source


def planner_source(source):
    source = held_planner(source)
    source = replace_once(source,
        "stage not in ('approach','excitation','excitation_hold','tracking','finish_hold','landing','complete')",
        "stage not in ('approach','excitation','preview_out','preview_return','excitation_hold','tracking','finish_hold','landing','complete')")
    source = replace_once(source,
        "elif stage=='excitation_hold' and self.cfg.get('hold_active_view_in_second_cycle',True):",
        "elif stage in ('preview_return','excitation_hold') and self.cfg.get('hold_active_view_in_second_cycle',True):")
    return source
