from glob import glob
from os.path import join

from setuptools import setup

package_name = 'odin_coordinator'

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
    description='Hostage event validation and rescue candidate coordination for ODIN-RESCUE.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rescue_coordinator = odin_coordinator.rescue_coordinator:main',
        ],
    },
)
