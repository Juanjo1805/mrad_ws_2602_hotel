from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner_2602_hotel'

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
    maintainer='juanjo',
    maintainer_email='juanjo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dijkstra_path_planner=hotel_path_planner.dijkstra_path_planner:main',
            'dijkstra_pp_2602_hotel=hotel_path_planner.dijkstra_pp_2602_hotel:main',
            'hybrid_astar_pp_2602_hotel=hotel_path_planner.hybrid_astar_pp_2602_hotel:main',
            'bitstar_planner_node=hotel_path_planner.bitstar_planner_node:main',
            'map_waypoint_mission=hotel_path_planner.map_waypoint_mission:main',
            'teb_controller_node=hotel_path_planner.teb_controller_node:main',
            'ara_path_planner=hotel_path_planner.ara_path_planner:main',
            'dijkstra_node=hotel_path_planner.dijkstra_node:main',
        ],
    },
)
