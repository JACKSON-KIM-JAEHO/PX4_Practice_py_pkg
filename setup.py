from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_practice_py_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [수정] 런치 파일: glob을 사용하여 launch 디렉토리의 모든 .launch.py 파일을 설치
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.py'))),
        
        # [BT XML] bt_xml 디렉토리의 모든 .xml 파일을 설치
        (os.path.join('share', package_name, 'bt_xml'), 
         glob(os.path.join('bt_xml', '*.xml'))),
    ],
    
    install_requires=['setuptools', 'py-trees', 'rclpy', 'mavros_msgs', 'geometry_msgs'], 
    # [참고] ROS2 메시지 및 rclpy는 빌드 의존성이지만, 설치 의존성에 명시하는 것이 좋음.
    
    zip_safe=True,
    maintainer='jaeho',
    maintainer_email='jaeho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #'main = px4_practice_py_pkg.main:main',
            'offboardtakeoff= px4_practice_py_pkg.offboardtakeoff:main',
        ],
    },
)
