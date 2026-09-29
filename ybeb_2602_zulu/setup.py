"""Install the two Pi RC executables and their configurations."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ybeb_2602_zulu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi5master',
    maintainer_email='rpi5master@todo.todo',
    description='Simple Raspberry RC AEB and Rosmaster interface',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'aeb_node = ybeb_2602_zulu.aeb_node:main',
            'ybeb_node = ybeb_2602_zulu.ybeb_node:main',
        ],
    },
)
