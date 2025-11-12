from setuptools import setup

package_name = 'wafer_stage_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Cartesian→wire controller and joint broadcaster',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wafer_controller = wafer_stage_control.wafer_controller:main',
            'wafer_joint_broadcaster = wafer_stage_control.wafer_joint_broadcaster:main',
            'wafer_wire_visualizer = wafer_stage_control.wafer_wire_visualizer:main',
            'wire_length_publisher = wafer_stage_control.wire_length_publisher:main',
        ],
    },
)
