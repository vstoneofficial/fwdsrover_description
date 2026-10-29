import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class IsolatedNode(Node):
    """Forward terminal shutdown to a child outside the terminal session."""

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=True)


def generate_launch_description():

    # --------------------------------------------------
    # Package path
    # --------------------------------------------------
    desc_pkg = get_package_share_directory('fwdsrover_description')

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    rover = LaunchConfiguration('rover', default='x40a')
    gui = LaunchConfiguration('gui', default='true')

    # --------------------------------------------------
    # Select xacro and rviz by rover type
    # --------------------------------------------------
    model_file = PythonExpression([
        repr(shlex.quote(os.path.join(desc_pkg, 'urdf') + os.sep)),
        ' + ("x40a.xacro" if "', rover,
        '" == "x40a" else ("x120a_lb.xacro" if "', rover,
        '" == "x120a_lb" else "x120a.xacro"))'
    ])

    rviz_file = PythonExpression([
        repr(os.path.join(desc_pkg, 'rviz') + os.sep),
        ' + ("x40a.rviz" if "', rover,
        '" == "x40a" else "x120a.rviz")'
    ])

    # --------------------------------------------------
    # Robot description
    # --------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', model_file]),
        value_type=str
    )

    # --------------------------------------------------
    # Nodes
    # --------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui = IsolatedNode(
        prefix='setsid',
        package='fwdsrover_description',
        executable='joint_state_gui.py',
        condition=IfCondition(gui)
    )

    rviz = IsolatedNode(
        prefix='setsid',
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'rover',
            default_value='x40a',
            description='Rover type: x40a, x120a, x120a_lb'
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
        ),

        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
