from setuptools import find_packages, setup

package_name = 'custom_dwa_local_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'custom_dwa_local_planner/config/custom_dwa_params.yaml'
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='umar',
    maintainer_email='mdumaribrahim91@gmail.com',
    description='Custom DWA local planner (from scratch) for TurtleBot3 in Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_dwa_node = custom_dwa_local_planner.custom_dwa_node:main',
        ],
    },
)
