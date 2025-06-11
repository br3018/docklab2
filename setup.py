import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'docklab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Rose',
    maintainer_email='ben.rose@orbitfab.com',
    description='Docklab 2 ROS 2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'relay_control6 = docklab2.relay_server6:main',
        'relay_control3 = docklab2.relay_server3:main',
        'GRASP_control = docklab2.GRASP_node:main',
        'base_control = docklab2.base_node:main'
        ],
    },
)
