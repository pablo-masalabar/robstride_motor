import glob
from setuptools import find_packages, setup

package_name = 'trajectory_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/trajectory_tracker']),
        ('share/trajectory_tracker',        ['package.xml']),
        ('share/trajectory_tracker/launch',         glob.glob('launch/*')),
        ('share/trajectory_tracker/config',         glob.glob('config/*')),
        ('share/trajectory_tracker/recorded_poses',        glob.glob('recorded_poses/*')),
        ('share/trajectory_tracker/recorded_trajectories', glob.glob('recorded_trajectories/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Trajectory tracking node for RobStride motors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'trajectory_tracker_node = trajectory_tracker.trajectory_tracker_node:main',
        ],
    },
)
