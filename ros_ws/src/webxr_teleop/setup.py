import glob
from setuptools import find_packages, setup

package_name = 'webxr_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/webxr_teleop']),
        ('share/webxr_teleop',        ['package.xml']),
        ('share/webxr_teleop/launch', glob.glob('launch/*')),
        ('share/webxr_teleop/config', glob.glob('config/*')),
        ('share/webxr_teleop/urdf/left_arm',  glob.glob('webxr_teleop/urdf/left_arm/*')),
        ('share/webxr_teleop/urdf/right_arm', glob.glob('webxr_teleop/urdf/right_arm/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='WebXR teleoperation node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'webxr_teleop_node = webxr_teleop.webxr_teleop_node:main',
        ],
    },
)
