import glob
from setuptools import find_packages, setup

package_name = 'remote_joystick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/remote_joystick']),
        ('share/remote_joystick', ['package.xml']),
        ('share/remote_joystick/launch', glob.glob('launch/*')),
        ('share/remote_joystick/config', glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Remote joystick bridge node for RobStride robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'remote_joystick_node = remote_joystick.remote_joystick_node:main',
        ],
    },
)
