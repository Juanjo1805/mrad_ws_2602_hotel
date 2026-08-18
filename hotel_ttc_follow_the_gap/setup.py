from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hotel_ttc_follow_the_gap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'), glob('config/*.*')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')),
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
                        'ttc_gap_finder=hotel_ttc_follow_the_gap.ttc_gap_finder:main',
                        'ttc_control=hotel_ttc_follow_the_gap.ttc_control:main',

        ],
    },
)
