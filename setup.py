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
        (os.path.join('share', package_name, 'bt_xml'), glob('bt_xml/*.xml')),
    ],
    


    install_requires=['setuptools', 'py-trees'],
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
            #'bt_runner = px4_practice_py_pkg.main:main',
            'main = px4_practice_py_pkg.main:main',
            'offboardtakeoff= px4_practice_py_pkg.offboardtakeoff:main',

        ],
    },
)
