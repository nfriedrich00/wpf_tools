import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wpf_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "include"), glob("include/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmc11',
    maintainer_email='nils-jonathan.friedrich@student.tu-freiberg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'ground_truth_publisher = wpf_tools.ground_truth_publisher_node:main',
                'path_plotter_localization = wpf_tools.path_plotter_odometry_node:main',
                'path_plotter_ground_truth = wpf_tools.path_plotter_ground_truth_node:main',
                'gps_error_simulator = wpf_tools.gps_error_simulator_node:main',
                'odom_republish = wpf_tools.odom_republish_node:main',
                'generate_waypoints = wpf_tools.waypoint_generator_node:main',
                'wait_for_localization = wpf_tools.wait_for_localization_node:main',
                'wait_for_navigation = wpf_tools.wait_for_navigation_node:main',
                'goal_checker = wpf_tools.goal_checker_node:main',
                'log_position = wpf_tools.log_position_node:main',
                'analyze_data = wpf_tools.analyzer_node:main',
                'start_topic_monitor = wpf_tools.monitor_node:main',
        ],
    },
)
