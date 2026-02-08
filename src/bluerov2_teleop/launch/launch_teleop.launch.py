import os
import pathlib
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # params = os.path.join(
    #     get_package_share_directory('bluerov2_teleop'),
    #     'params',
    #     'params.yaml'
    # )

    config_dir = os.path.join(get_package_share_directory('bluerov2_teleop'), 'params')
    param_config = os.path.join(config_dir, "params.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["bluerov2_teleop_node"]["ros__parameters"]


    bluerov2_teleop_node = Node(
        package = 'bluerov2_teleop',
        executable = 'bluerov2_teleop_exe',
        output ='screen',
        parameters = [params]
    )


    ld.add_action(bluerov2_teleop_node)

    return ld