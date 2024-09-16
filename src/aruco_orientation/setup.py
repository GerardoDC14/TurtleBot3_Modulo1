from setuptools import setup

package_name = 'aruco_orientation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'aruco_orientation.odom_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 node for handling robot orientation and position',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = aruco_orientation.odom_node:main',
        ],
    },
)
