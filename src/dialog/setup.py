from setuptools import setup

package_name = 'dialog'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/llm_mock.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='val',
    maintainer_email='val@todo.todo',
    description='Mock LLM interaction action server/client (ROS2).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_mock_server = dialog.llm_mock_server:main',
            'llm_mock_client = dialog.llm_mock_client:main',
        ],
    },
)
