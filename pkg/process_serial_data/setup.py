from setuptools import find_packages, setup

package_name = 'process_serial_data'

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
    maintainer='wangxiaotao',
    maintainer_email='hliu6076@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'unload_serial_data = process_serial_data.unload_serial_data:main',
            'pub_target_pose = process_serial_data.pub_target_pose:main',
            'pub_gripper_control = process_serial_data.pub_gripper_control:main',
            'tf_broadcast = process_serial_data.tf_broadcast:main',
            'pub_target_pose_imu = process_serial_data.pub_target_pose_imu:main',
            'pub_target_pose_imu_multi = process_serial_data.pub_target_pose_imu_multi:main'
        ],
    },
)
