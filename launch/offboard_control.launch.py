from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_practice_py_pkg',
            executable='offboardtakeoff',
            name='offboardtakeoff',
            output='screen',
            parameters=[]   # 고도·속도 파라미터화는 추후 추가
        )
    ])