"""Display the synthetic tunnel only, with an interior camera and WSL setup."""
import os
from pathlib import Path
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    layout = LaunchConfiguration('layout').perform(context)
    workspace = Path(os.environ.get('MULTILINK_AUTONOMY_WS', '/root/multilink_autonomy_ws'))
    explicit_world = LaunchConfiguration('world_file').perform(context)
    world = Path(explicit_world) if explicit_world else workspace / 'assets/switching_tunnel_20261024' / (layout + '.sdf')
    if not world.is_file():
        raise RuntimeError(f'Tunnel world not found: {world}')

    runtime = Path('/tmp') / f'uas-gazebo-runtime-{os.getuid()}'
    runtime.mkdir(mode=0o700, exist_ok=True)
    runtime.chmod(0o700)
    temporary = tempfile.TemporaryDirectory(prefix='tunnel-view-', dir=runtime)
    simulation = Path(get_package_share_directory('aerial_robot_simulation'))
    config = (simulation / 'config/gazebo_gui.config').read_text()
    camera_pose = LaunchConfiguration('camera_pose').perform(context)
    pose = [float(x) for x in camera_pose.split()]
    if len(pose) != 6:
        raise ValueError('camera_pose must have six numbers')
    config = re.sub(r'<camera_pose>.*?</camera_pose>',
                    '<camera_pose>' + ' '.join(map(str, pose)) + '</camera_pose>', config)
    gui = Path(temporary.name) / 'gui.config'
    gui.write_text(config)

    env = {
        'XDG_RUNTIME_DIR': str(runtime),
        'GZ_PARTITION': f'tunnel_view_{os.getpid()}',
        'GZ_GUI_PLUGIN_PATH': str(simulation.parent.parent / 'lib') + os.pathsep + os.environ.get('GZ_GUI_PLUGIN_PATH', ''),
    }
    if Path('/usr/lib/wsl/lib/libd3d12core.so').is_file():
        env.update(GALLIUM_DRIVER='d3d12', LIBGL_ALWAYS_SOFTWARE='0',
                   MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA', QT_QPA_PLATFORM='xcb',
                   QT_X11_NO_MITSHM='1', MESA_GL_VERSION_OVERRIDE='3.3',
                   MESA_GLSL_VERSION_OVERRIDE='330',
                   LD_PRELOAD='/usr/lib/wsl/lib/libd3d12core.so' +
                   (' ' + os.environ['LD_PRELOAD'] if os.environ.get('LD_PRELOAD') else ''))

    def cleanup(_context):
        temporary.cleanup()
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        ExecuteProcess(cmd=['gz', 'sim', '--render-engine', 'ogre2',
                            '--gui-config', str(gui), str(world)],
                       additional_env=env, output='screen',
                       on_exit=[Shutdown(reason='Tunnel viewer closed')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='alternating',
                              choices=['alternating', 'reversed', 'uniform'],
                              description='Texture distribution in the tunnel'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('camera_pose', default_value='42 0 2.5 0 0 0.6'),
        OpaqueFunction(function=setup),
    ])
