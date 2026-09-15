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
        ('share/' + package_name + '/config', [
            'config/optimization_baseline.yaml',
            'config/adaptive_pure_pursuit.yaml',
            'config/manual_adaptive_double_speed.yaml',
        ]),
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
            'adaptive_pure_pursuit=hotel_path_tracking.adaptive_pure_pursuit_node:main',
            'publish_initial_pose=hotel_path_tracking.publish_initial_pose:main',
            'optimization_monitor=hotel_path_tracking.optimization_monitor:main',
            'run_optimization=hotel_path_tracking.run_optimization:main',
            'record_manual_baseline=hotel_path_tracking.record_manual_baseline:main',
            'compare_baselines=hotel_path_tracking.compare_baselines:main',
            'compare_run001_vs_run002=hotel_path_tracking.compare_run001_vs_run002:main',
            'analyze_baseline_reproducibility=hotel_path_tracking.analyze_baseline_reproducibility:main',
            'system_readiness=hotel_path_tracking.system_readiness:main',
            'analyze_optimization=hotel_path_tracking.analyze_optimization:main',
            'lqr_pt_2602_hotel=hotel_path_tracking.lqr_pt_2602_hotel:main',
            'dwb_path_tracker=hotel_path_tracking.dwb_path_tracker:main',
            'stanley_node=hotel_path_tracking.stanley_node:main',
        ],
    },
)
