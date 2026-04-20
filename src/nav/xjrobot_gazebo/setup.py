import os
from glob import glob
from setuptools import setup

package_name = 'xjrobot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds', 'lino'), glob(os.path.join('worlds', 'lino', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds', 'xjrobot'), glob(os.path.join('worlds', 'xjrobot', '*.sdf'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'hook'), glob(os.path.join('hook', '*.sh'))),
        *[
            (os.path.join('share', package_name, os.path.dirname(file_path)), [file_path])
            for file_path in glob(os.path.join('models', '**', '*'), recursive=True)
            if os.path.isfile(file_path)
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wu',
    maintainer_email='wu@todo.todo',
    description='Gazebo simulation package for xjrobot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
