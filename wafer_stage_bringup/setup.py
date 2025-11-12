from setuptools import setup

package_name = 'wafer_stage_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wafer_stage_bringup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/webui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Launch files for wafer stage',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
