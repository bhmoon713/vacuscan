from setuptools import setup
package_name = 'vacuscan_webui'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', ['web/index.html', 'web/app.js']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Static web UI for wafer stage (use with rosbridge)',
    license='Apache-2.0',
)
