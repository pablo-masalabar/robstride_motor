import glob
from setuptools import find_packages, setup

package_name = 'damiao_p'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/damiao_p']),
        ('share/damiao_p',              ['package.xml']),
        ('share/damiao_p/launch',       glob.glob('launch/*')),
        ('share/damiao_p/config',       glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='ROS2 driver node for Damiao motors over CAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = damiao_p.motor_node:main',
        ],
    },
)
