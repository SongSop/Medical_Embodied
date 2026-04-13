from setuptools import setup

package_name = 'monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/monitor_mock.launch.py', 'launch/face_identify.launch.py']),
        ('share/' + package_name + '/face_database', ['face_database/README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='val',
    maintainer_email='val@todo.todo',
    description='Mock monitor publishers and detection services (ROS2).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_mock_pub = monitor.monitor_mock_pub:main',
            'anomaly_detect_server = monitor.anomaly_detect_server:main',
            'face_identify_server = monitor.face_identify_server:main',
        ],
    },
)
