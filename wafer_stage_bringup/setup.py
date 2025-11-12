from setuptools import setup
from glob import glob
import os

package_name = 'wafer_stage_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # <-- no python package installed
    data_files=[
        # Required index entries
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install ALL launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install ALL RViz configs
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
        # (optional) config files like waypoints.yaml
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup launch and visualization for wafer stage',
    license='Apache-2.0',
)
