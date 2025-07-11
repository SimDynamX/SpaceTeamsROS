from setuptools import find_packages, setup

package_name = 'space_teams_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yotam',
    maintainer_email='yotam@todo.todo',
    description='Python ROS2 package for SpaceTeams project with logger_info service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_client = space_teams_python.example_client:main',
        ],
    },
)
