from setuptools import setup

package_name = 'charge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/charge_services.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='val',
    maintainer_email='val@todo.todo',
    description='Mock docking and charging services (ROS2).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charge_services = charge.charge_services:main',
        ],
    },
)
