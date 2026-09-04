from setuptools import find_packages, setup

package_name = 'path_tracker_2602_hotel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'pure_pursuit_node=hotel_path_tracking.pure_pursuit_node:main',
            'pure_pursuit_pt_2602_hotel=hotel_path_tracking.pure_pursuit_node:main_2602',
            'lqr_pt_2602_hotel=hotel_path_tracking.lqr_pt_2602_hotel:main',
            'dwb_path_tracker=hotel_path_tracking.dwb_path_tracker:main',
            'stanley_node=hotel_path_tracking.stanley_node:main',
        ],
    },
)
