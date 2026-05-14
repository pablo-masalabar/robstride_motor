import glob
from setuptools import find_packages, setup

package_name = 'teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/teleop']),
        ('share/teleop',        ['package.xml']),
        ('share/teleop/launch', glob.glob('launch/*')),
        ('share/teleop/config', glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Teleoperation node for RobStride motors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_node = teleop.teleop_node:main',
            'joy_reader  = teleop.joy_reader:main',
        ],
    },
)
