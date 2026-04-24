from setuptools import setup

package_name = 'nurse_call'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/call_nurse_mock.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='val',
    maintainer_email='val@todo.todo',
    description='Mock call nurse action server/client (ROS2).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'call_nurse_mock_server = nurse_call.call_nurse_mock_server:main',
            'call_nurse_mock_client = nurse_call.call_nurse_mock_client:main',
        ],
    },
)
