from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_practice_py_pkg'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        # ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # ✅ launch 파일
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))
        ),

        # ✅ fire 시나리오 BT XML
        (
            os.path.join(
                'share',
                package_name,
                'scenario',
                'fire'
            ),
            glob(
                os.path.join(
                    'px4_practice_py_pkg',
                    'scenario',
                    'fire',
                    '*.xml'
                )
            )
        ),

        # ✅ fire 시나리오 YAML
        (
            os.path.join(
                'share',
                package_name,
                'scenario',
                'fire',
                'config'
            ),
            glob(
                os.path.join(
                    'px4_practice_py_pkg',
                    'scenario',
                    'fire',
                    'config',
                    '*.yaml'
                )
            )
        ),
    ],

    install_requires=[
        'setuptools',
        'py_trees',
        'rclpy',
        'mavros_msgs',
        'geometry_msgs'
    ],

    zip_safe=True,
    maintainer='jaeho',
    maintainer_email='jaeho@todo.todo',
    description='PX4 + BehaviorTree practice package',
    license='TODO',

    entry_points={
        'console_scripts': [
            'fire_bt = px4_practice_py_pkg.scenario.fire.main:main',
        ],
    },
)
