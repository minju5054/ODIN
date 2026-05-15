from glob import glob
from os.path import join

from setuptools import setup

package_name = 'odin_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odin',
    maintainer_email='odin@example.com',
    description='Nav2 and frontier exploration launch files for ODIN-RESCUE.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_3_spawn_on_goal = odin_navigation.robot_3_spawn_on_goal:main',
            'simple_goal_follower = odin_navigation.simple_goal_follower:main',
            'nav2_goal_dispatcher = odin_navigation.nav2_goal_dispatcher:main',
            'mission_success_marker = odin_navigation.mission_success_marker:main',
            'mission_success_popup = odin_navigation.mission_success_popup:main',
        ],
    },
)
