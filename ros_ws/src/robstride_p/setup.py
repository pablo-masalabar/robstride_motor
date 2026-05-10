from setuptools import find_packages, setup

package_name = 'robstride_p'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robstride_p']),
        ('share/robstride_p',              ['package.xml']),
        ('share/robstride_p/launch',       ['launch/motors.launch.py']),
        ('share/robstride_p/config',       ['config/config.toml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='ROS2 driver node for RobStride motors over CAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = robstride_p.motor_node:main',
        ],
    },
)
