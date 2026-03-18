from setuptools import find_packages, setup

package_name = 'wlkata_arm_move'

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
    maintainer='wlkata',
    maintainer_email='wlkata@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'harobot_moveit_move=wlkata_arm_move.harobot_moveit_move:main',
            'harobot_server=wlkata_arm_move.harobot_server:main',
            'harobot_client=wlkata_arm_move.harobot_client:main',
            'mirobot_moveit_move=wlkata_arm_move.mirobot_moveit_move:main',
            'mt4_moveit_move=wlkata_arm_move.mt4_moveit_move:main',
        ],
    },
)
