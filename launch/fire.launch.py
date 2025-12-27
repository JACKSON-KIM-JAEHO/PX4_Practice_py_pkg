import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 share 디렉토리
    pkg_share = get_package_share_directory('px4_practice_py_pkg')

    # 🔥 fire 시나리오 YAML 경로
    config_file = os.path.join(
        pkg_share,
        'scenario',
        'fire',
        'config',
        'offboard_takeoff.yaml'
    )

    bt_runner_node = Node(
        package='px4_practice_py_pkg',
        executable='fire_bt',      # setup.py console_scripts
        name='bt_runner',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        bt_runner_node
    ])
