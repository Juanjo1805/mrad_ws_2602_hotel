from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hotel_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'diffdrive_urdf'), glob('diffdrive_urdf/*.*')),
        (os.path.join('share', package_name, 'ackerman_urdf'), glob('ackerman_urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
