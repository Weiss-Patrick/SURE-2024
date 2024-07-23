from setuptools import setup

package_name = 'light_follower_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'light_follower_controller.light_sensing_node',
        'light_follower_controller.motor_control_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A light-following robot using ROS 2',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'light_sensing_node = light_follower_controller.light_sensing_node:main',
            'motor_control_node = light_follower_controller.motor_control_node:main'
        ],
    },
)
