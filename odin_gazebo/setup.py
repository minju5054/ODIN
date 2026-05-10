from glob import glob
from os.path import join

from setuptools import setup

package_name = 'odin_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (join('share', package_name, 'models', 'house'), glob('models/house/*')),
        (
            join('share', package_name, 'models', 'aruco_marker_0'),
            [
                'models/aruco_marker_0/model.config',
                'models/aruco_marker_0/model.sdf',
            ],
        ),
        (
            join('share', package_name, 'models', 'aruco_marker_0', 'materials', 'scripts'),
            glob('models/aruco_marker_0/materials/scripts/*'),
        ),
        (
            join('share', package_name, 'models', 'aruco_marker_0', 'materials', 'textures'),
            glob('models/aruco_marker_0/materials/textures/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odin',
    maintainer_email='odin@example.com',
    description='Gazebo Classic worlds and multi-robot spawn launch files for ODIN.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
