from glob import glob
from os.path import join

from setuptools import setup

package_name = 'odin_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'config'), glob('config/*.yaml')),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odin',
    maintainer_email='odin@example.com',
    description='AI waypoint recommendation bridge and local heuristic fallback for ODIN-RESCUE.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heuristic_waypoint_recommender = odin_ai.heuristic_waypoint_recommender:main',
            'mission_intent_panel = odin_ai.mission_intent_panel:main',
            'virtual_qwen_planner = odin_ai.virtual_qwen_planner:main',
        ],
    },
)
