import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('robot_4wd')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    use_nvidia = LaunchConfiguration('use_nvidia').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(pkg, 'worlds', world)

    # xacro -> URDF string
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    bridge_config = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    sim_time = {'use_sim_time': use_sim_time}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, **sim_time}],
    )

    # '-s' = server only (headless) when gui:=false
    gz_flags = '-r -v4' if gui else '-s -r -v4'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{gz_flags} {world}'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'swerve_bot', '-z', '0.1'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, **sim_time}],
    )

    swerve = Node(
        package='robot_4wd',
        executable='swerve_controller',
        output='screen',
        parameters=[sim_time],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'config', 'view.rviz')]
        if os.path.exists(os.path.join(pkg, 'config', 'view.rviz')) else [],
        parameters=[sim_time],
    )

    actions = []
    if use_nvidia:
        # On NVIDIA Optimus/PRIME laptops the Gazebo Sensors system fails to
        # create an offscreen EGL context on the discrete GPU (libEGL "failed
        # to create dri2 screen") -> gpu_lidar produces no data. Force the
        # offscreen EGL onto the NVIDIA vendor so /scan renders.
        actions += [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                                   '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        ]
    actions += [rsp, gz_sim, spawn, bridge, swerve, rviz]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='true = Gazebo GUI, false = headless server'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='also open RViz2'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='world file (name in worlds/ or absolute path)'),
        DeclareLaunchArgument('use_nvidia', default_value='true',
                              description='force offscreen EGL onto the NVIDIA GPU '
                                          '(needed on Optimus laptops for gpu_lidar). '
                                          'Set false on non-NVIDIA machines.'),
        OpaqueFunction(function=launch_setup),
    ])
