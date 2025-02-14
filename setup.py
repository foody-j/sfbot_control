from setuptools import find_packages, setup

package_name = 'sfbot_control'

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
    maintainer='yj',
    maintainer_email='heyyoungjin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_client = sfbot_control.joint_trajectory_client_six:main',
            'keyboard_joint_controller = sfbot_control.keyboard_joint_controller:main',
            'robot_arm_controller = sfbot_control.robot_arm_controller:main',
            'circle_controll = sfbot_control.circle_controll:main',
            'keyborad = sfbot_control.foward_keyborad_controller:main',
        ],
    },
)
