from setuptools import find_packages, setup

package_name = 'llm_node_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/key', ['key/dashscope.key']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='2657979822@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_node_nurse_alert = llm_node_py.ros_node_nurse_alert:main',
            'ros_node_question_router = llm_node_py.ros_node_question_router:main',
            'ros_node_tts_oneshot = llm_node_py.ros_node_tts_oneshot:main',
            'ros_node_tts_realtime = llm_node_py.ros_node_tts_realtime:main',
        ],
    },
)
