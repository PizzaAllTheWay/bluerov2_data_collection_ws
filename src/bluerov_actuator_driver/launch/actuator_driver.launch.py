import os
import pathlib
import yaml
import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action

import lifecycle_msgs.msg



def generate_launch_description():
    # Get the bringup directory
    # bringup_dir = FindPackageShare('bluerov_actuator_driver').find('bluerov_actuator_driver')

    # Set parameter file path
    # param_file_path = os.path.join(bringup_dir, 'params', 'params.yaml')
    # param_file = launch.substitutions.LaunchConfiguration('params', default=[param_file_path])

    config_dir = os.path.join(get_package_share_directory('bluerov_actuator_driver'), 'params')
    param_config = os.path.join(config_dir, "params.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["bluerov_actuator_driver"]["ros__parameters"]

    # Create the launch configuration variables
    # autostart_param = DeclareLaunchArgument(
    #     name='autostart',
    #     default_value='True',
    #     description='Automatically start lifecycle nodes')
    # priority_param = DeclareLaunchArgument(
    #     name='priority',
    #     default_value='0',
    #     description='Set process priority')
    # cpu_affinity_param = DeclareLaunchArgument(
    #     name='cpu-affinity',
    #     default_value='0',
    #     description='Set process CPU affinity')
    # with_lock_memory_param = DeclareLaunchArgument(
    #     name='lock-memory',
    #     default_value='False',
    #     description='Lock the process memory')
    # lock_memory_size_param = DeclareLaunchArgument(
    #     name='lock-memory-size',
    #     default_value='0',
    #     description='Set lock memory size in MB')
    # config_child_threads_param = DeclareLaunchArgument(
    #     name='config-child-threads',
    #     default_value='False',
    #     description='Configure process child threads (typically DDS threads)')

    # # Node definitions
    # bluerov_actuator_driver_node = Node(
    #     package='bluerov_actuator_driver',
    #     executable='bluerov_actuator_driver_exe',
    #     output='screen',
    #     parameters=[params]
    #     # arguments=[
    #       #  '--autostart', LaunchConfiguration('autostart')
    #     #    '--priority', LaunchConfiguration('priority'),
    #     #    '--cpu-affinity', LaunchConfiguration('cpu-affinity'),
    #     #    '--lock-memory', LaunchConfiguration('lock-memory'),
    #     #    '--lock-memory-size', LaunchConfiguration('lock-memory-size'),
    #     #    '--config-child-threads', LaunchConfiguration('config-child-threads')
    #       #  ]
    # )

    # ld = LaunchDescription()

    # ld.add_action(autostart_param)
    # ld.add_action(priority_param)
    # ld.add_action(cpu_affinity_param)
    # ld.add_action(with_lock_memory_param)
    # ld.add_action(lock_memory_size_param)
    # ld.add_action(config_child_threads_param)
    # ld.add_action(bluerov_actuator_driver_node)

    ld = LaunchDescription()

    # Node definitions
    bluerov_actuator_driver_node = LifecycleNode(
        package='bluerov_actuator_driver',              # must match name in config -> YAML
        executable='bluerov_actuator_driver_exe',
        name='actuator_driver',                            # must match node name in config -> YAML
        output='screen',
        namespace='bluerov_actuator_driver',
        parameters=[params]
    )


    # Create the launch configuration variables
    # Make the node take the 'configure' transition
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(bluerov_actuator_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the node take the 'activate' transition
    # activate_event = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=bluerov_actuator_driver_node, goal_state='inactive',
    #         entities=[
    #             LogInfo(
    #                 msg="[LifecycleLaunch] bluerov_actuator_driver node is activating."),
    #             EmitEvent(event=ChangeState(
    #                 lifecycle_node_matcher=matches_action(bluerov_actuator_driver_node),
    #                 transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
    #             )),
    #         ],
    #     )
    # )
    activate_event = EmitEvent(
      event=ChangeState(
          lifecycle_node_matcher=matches_action(bluerov_actuator_driver_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
      )
    )


    ld.add_action(bluerov_actuator_driver_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)


    return ld