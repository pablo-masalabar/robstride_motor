import glob
from setuptools import find_packages, setup

package_name = 'ezmotion_p'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ezmotion_p']),
        ('share/ezmotion_p',        ['package.xml']),
        ('share/ezmotion_p/launch', glob.glob('launch/*.py')),
        ('share/ezmotion_p/config', glob.glob('config/*.toml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='EzMotion node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ezmotion_node = ezmotion_p.motor_node:main',
        ],
    },
)
