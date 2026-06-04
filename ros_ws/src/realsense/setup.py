import glob
from setuptools import find_packages, setup

package_name = 'realsense'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/realsense']),
        ('share/realsense',        ['package.xml']),
        ('share/realsense/launch', glob.glob('launch/*')),
        ('share/realsense/config', glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='RealSense camera node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'realsense_node = realsense.realsense_node:main',
        ],
    },
)
