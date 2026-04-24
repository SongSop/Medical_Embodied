from setuptools import setup

package_name = 'loadconfig'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
        ('share/' + package_name + '/launch', ['launch/loadconfig.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='val',
    maintainer_email='val@todo.todo',
    description='Configuration loader and parameter manager (ROS2).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loadconfig_server = loadconfig.loadconfig_server:main',
        ],
    },
)
