import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hotel_line_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanse',
    maintainer_email='the.jj.65.gy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_extractor_node=hotel_line_extractor.line_extractor_node:main',
            'lka_node=hotel_line_extractor.lka_node:main',
        ],
    },
)
