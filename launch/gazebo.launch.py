"""Run a 4WDS model in ROS 2 Jazzy / Gazebo Harmonic."""
import asyncio
import os
import shutil
import signal
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueCoroutine, OpaqueFunction, Shutdown,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


class DrainBeforeShutdown:
    """Defer the normal launch signal until the producer has drained."""
    shutdown_ready = False

    def _shutdown_process(self, context, *, send_sigint):
        # Preserve launch cancellation for a process that has not started.
        if self.process_details is None or self.process_details.get('pid') is None:
            self.shutdown_ready = True
        if not self.shutdown_ready:
            return None
        return super()._shutdown_process(context, send_sigint=True)


class GazeboProcess(DrainBeforeShutdown, ExecuteProcess):
    pass


class PointCloudBridge(DrainBeforeShutdown, Node):
    pass

def setup(context):
    rover = LaunchConfiguration('rover').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    package = get_package_share_directory('fwdsrover_description')
    options = get_package_share_directory('vs_rover_options_description')
    description = xacro.process_file(os.path.join(package, 'urdf', rover + '.xacro')).toxml()
    gz = shutil.which('gz')
    partition = context.environment.get('GZ_PARTITION') or 'fwds_' + uuid.uuid4().hex
    # Separate sessions prevent terminal SIGINT from bypassing the drain phase.
    # exec preserves the native PID under launch's normal process supervision.
    native_command = [sys.executable, '-c',
                      'import os, sys; os.setsid(); os.execv(sys.argv[1], sys.argv[1:])',
                      gz, 'sim']

    def shutdown_if_running(context):
        # A second Shutdown cancels an active OpaqueCoroutine in Jazzy.
        return [] if context.is_shutdown else [Shutdown()]

    gazebo = GazeboProcess(
        cmd=native_command + ['-r', '-s'] +
            [os.path.join(package, 'worlds', 'empty.sdf'), '--force-version', '8'],
        shell=False, name='gazebo', output='screen',
        on_exit=OpaqueFunction(function=shutdown_if_running))
    gui_actions = [GazeboProcess(
        cmd=native_command + ['-g', '--force-version', '8'], shell=False,
        name='gazebo_gui', output='screen',
        on_exit=OpaqueFunction(function=shutdown_if_running))] if gui else []
    native_processes = [gazebo, *gui_actions]

    async def stop_gazebo(context):
        def pid_of(action):
            return action.process_details.get('pid') if action.process_details else None

        def alive(pid):
            if pid is None:
                return False
            try:
                with open(f'/proc/{pid}/stat') as status:
                    return not status.read().rpartition(') ')[2].startswith('Z ')
            except FileNotFoundError:
                return False

        if not any(alive(pid_of(action)) for action in native_processes + point_bridges):
            return
        logger = get_logger('fwds_shutdown')
        env = dict(context.environment)
        client = None
        # Qt ignores quit() before its event loop starts. Keep the server
        # available while the GUI initializes and retry only the GUI signal.
        # Repeating the server signal can re-enter its synchronous Stop.
        for action in gui_actions:
            if alive(pid_of(action)):
                action.shutdown_ready = True
                await context.emit_event(ShutdownProcess(process_matcher=matches_action(action)))
            deadline = time.monotonic() + 5
            next_signal = time.monotonic() + .5
            while alive(pid_of(action)) and time.monotonic() < deadline:
                if time.monotonic() >= next_signal:
                    pid = pid_of(action)
                    logger.info(f'Requesting Gazebo GUI shutdown (SIGINT, PID {pid})')
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        break
                    next_signal = time.monotonic() + .5
                await asyncio.sleep(.01)
        deadline = time.monotonic() + 5
        try:
            if alive(pid_of(gazebo)):
                logger.info('Pausing Gazebo and draining pending sensor updates')
                client = await asyncio.create_subprocess_exec(
                    sys.executable, os.path.join(package, 'params', 'gazebo_shutdown.py'),
                    str(deadline), env=env, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE)
                stdout, stderr = await asyncio.wait_for(
                    client.communicate(), timeout=max(.01, deadline - time.monotonic()))
                if client.returncode:
                    raise RuntimeError((stdout + stderr).decode(errors='replace'))
                logger.info(stdout.decode().strip())
        except (OSError, RuntimeError, asyncio.TimeoutError) as error:
            logger.error(f'Gazebo shutdown preparation failed: {error}')
            if client is not None and client.returncode is None:
                client.kill()
                await client.wait()
        # Keep PointCloud callbacks alive until the native producer has drained.
        # Start each node's normal SIGINT/escalation path only at this point.
        for action in point_bridges:
            if alive(pid_of(action)):
                action.shutdown_ready = True
                logger.info(f'Stopping drained PointCloud bridge (PID {pid_of(action)})')
                await context.emit_event(ShutdownProcess(process_matcher=matches_action(action)))
        deadline = time.monotonic() + 5
        while (any(alive(pid_of(action)) for action in point_bridges)
               and time.monotonic() < deadline):
            await asyncio.sleep(.01)
        if any(alive(pid_of(action)) for action in point_bridges):
            logger.error('PointCloud bridges did not complete their normal shutdown within five seconds')
        # Use the main-thread signal path. ServerControl's asynchronous Stop
        # can race destruction and join the rendering thread twice in Sim 8.15.
        deadline = time.monotonic() + 5
        for action in [gazebo]:
            pid = pid_of(action)
            if alive(pid):
                action.shutdown_ready = True
                logger.info(f'Sending one SIGINT to {action.process_details["name"]} (PID {pid})')
                await context.emit_event(ShutdownProcess(process_matcher=matches_action(action)))
        while (any(alive(pid_of(action)) for action in native_processes)
               and time.monotonic() < deadline):
            await asyncio.sleep(.01)
        if any(alive(pid_of(action)) for action in native_processes):
            logger.error('Gazebo did not complete its normal shutdown within five seconds')
        else:
            logger.info('Gazebo stopped after sensor updates drained')
    publisher = Node(prefix='setsid',
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(description, value_type=str),
                     'use_sim_time': True}], output='screen')
    spawn = Node(prefix='setsid', package='ros_gz_sim', executable='create',
                 arguments=['-name', rover, '-topic', 'robot_description', '-z', '0.04'],
                 output='screen')
    relay = Node(prefix='setsid', package='fwdsrover_description', executable='rover_twist_relay.py',
                 parameters=[{'robot_description': ParameterValue(description, value_type=str),
                              'use_sim_time': True}], output='screen')
    topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/back_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/camera/infra1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/infra2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    ]
    for prefix in ('front_left', 'front_right', 'back_right', 'back_left'):
        for kind in ('wheel', 'steering'):
            topics.append(f'/fwds/{prefix}_{kind}_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double')
    model = ET.fromstring(description)
    bridge_remappings = []
    for sensor in model.findall('gazebo/sensor'):
        topic = sensor.findtext('topic')
        if sensor.get('type') == 'rgbd_camera' and topic:
            base = topic.rstrip('/')
            bridge_remappings.append((base + '/depth_image', base + '/image_raw'))
        elif sensor.get('type') == 'camera':
            info = sensor.findtext('camera/camera_info_topic')
            declaration = info + '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' if info else None
            if declaration and declaration not in topics:
                topics.append(declaration)
    bridge = Node(prefix='setsid', package='ros_gz_bridge', executable='parameter_bridge', arguments=topics,
                  remappings=bridge_remappings,
                  parameters=[{'use_sim_time': True}], output='screen')
    point_bridges = []
    # Sensors 8.2.2 labels native forward-X points with its image optical ID.
    # Keep the payload intact and use the point frame declared by the URDF.
    for sensor in model.findall("gazebo/sensor[@type='rgbd_camera']"):
        base = sensor.findtext('topic').rstrip('/')
        topic = base + '/points'
        optical_frame = sensor.findtext('camera/optical_frame_id')
        point_frame = next(
            (joint.find('parent').get('link') for joint in model.findall('joint')
             if joint.find('child').get('link') == optical_frame), sensor.findtext('gz_frame_id'))
        point_bridges.append(PointCloudBridge(prefix='setsid',
            package='ros_gz_bridge', executable='parameter_bridge',
            name=sensor.get('name') + '_points_bridge',
            arguments=[topic+'@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[(topic, base + '/color/points')],
            parameters=[{'use_sim_time': True,
                         'override_frame_id': point_frame}],
            output='screen'))
    return [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(package)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(options)),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[relay])),
        gazebo, publisher, bridge, spawn, *point_bridges, *gui_actions,
        # Handlers prepend in Jazzy: register after gazebo so this runs first.
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueCoroutine(coroutine=stop_gazebo)], handle_once=True)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rover', default_value='x40a',
                              choices=['x40a', 'x120a', 'x120a_lb']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
