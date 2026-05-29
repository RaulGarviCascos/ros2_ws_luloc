from setuptools import find_packages, setup

package_name = 'luloc_pkg_py'

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
    maintainer='raulgarvicascos',
    maintainer_email='raul.garvi.cascos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'cmd_vel_2_mqtt = luloc_pkg_py.cmd_vel_2_mqtt:main',
                'ros_msg_2_mqtt = luloc_pkg_py.ros_msg_2_mqtt:main',
                'servo_suscriber = luloc_pkg_py.servo_suscriber:main',
                'cam_control = luloc_pkg_py.cam_control:main',
        ],
    },
)
