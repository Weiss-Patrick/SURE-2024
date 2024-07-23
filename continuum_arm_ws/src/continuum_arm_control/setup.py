from setuptools import setup

package_name = 'continuum_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'dynamixel_sdk', 'rclpy', 'sensor_msgs', 'time'],
    zip_safe=True,
    maintainer='P-Dawg',
    maintainer_email='wpw00001@mix.wvu.edu',
    description='ackage enables basic veolcity control of a tendon-driven continuum arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = continuum_arm_control.arm_controller:main'
        ],
    },
)
