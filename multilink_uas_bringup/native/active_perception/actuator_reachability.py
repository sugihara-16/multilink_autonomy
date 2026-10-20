"""Read actual Gazebo position-controller speed limits; never change the model."""
import math
import xml.etree.ElementTree as ET


def parameters(model_path, world_path, stop_steps=2):
    if stop_steps not in (1,2):raise ValueError('Unknown hard-stop clearance model')
    model=ET.parse(model_path).getroot().find('model')
    world=ET.parse(world_path).getroot().find('world')
    step=float(world.findtext('physics/max_step_size'))
    if not math.isfinite(step) or step<=0:raise ValueError('Invalid physics step')
    rates=[]
    for axis in ('yaw','pitch'):
        name='active_camera_'+axis+'_joint'
        joint=model.find("joint[@name='"+name+"']")
        plugins=[p for p in model.findall('plugin') if p.findtext('joint_name')==name
                 and p.get('name')=='gz::sim::systems::JointPositionController']
        if joint is None or len(plugins)!=1:raise ValueError('Missing unique joint controller: '+name)
        p=plugins[0]
        if p.findtext('use_velocity_commands')!='true':raise ValueError('Unsupported joint controller mode')
        rate=float(p.findtext('cmd_max'));physical=float(joint.findtext('axis/limit/velocity'))
        if not (math.isfinite(rate) and math.isfinite(physical) and rate>0 and physical>0):
            raise ValueError('Invalid joint speed')
        rates.append(min(rate,physical))
    # Keep one additional step beyond the observed one-step hard-stop snap.
    # This is a numerical operating clearance, not a change to joint limits.
    return dict(axis_max_speed=rates,joint_stop_margin=[stop_steps*v*step for v in rates],joint_stop_steps=stop_steps)
