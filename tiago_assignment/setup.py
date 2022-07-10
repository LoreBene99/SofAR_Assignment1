from setuptools import setup

package_name = 'tiago_assignment'
data_files = []

data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)

data_files.append(('share/' + package_name + '/launch', ['launch/assignment_launch.py']))

data_files.append(('share/' + package_name + '/maps', [
    'maps/house.posegraph',
    'maps/house.data'
]))

data_files.append(('share/' + package_name + '/environments', [
    'environments/break_room_1.wbt', 
    'environments/.break_room_1.wbproj', 
    'environments/break_room_2.wbt', 
    'environments/.break_room_2.wbproj', 
    'environments/house.wbt', 
    'environments/.house.wbproj',
    'environments/warehouse.wbt', 
    'environments/.warehouse.wbproj'
]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/tiago_webots.urdf',
    'resource/ros2_control.yml',
    'resource/default.rviz',
    'resource/map.pgm',
    'resource/map.yaml'
]))

data_files.append(('share/' + package_name + '/configurations', [
    'configurations/navigation2_parameters.yaml',
    'configurations/slam_parameters.yaml', 
    'configurations/localization_parameters.yaml', 
    'configurations/lifelong_parameters.yaml'
]))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.2.2',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples', 'TIAGo'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TIAGo robots ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
