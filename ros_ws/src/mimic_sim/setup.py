from setuptools import find_packages, setup

package_name = 'mimic_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/config.toml',
            'config/sim_left_arm.toml',
            'config/sim_right_arm.toml',
            'config/sim_neck.toml',
            'config/sim_base_brackets.toml',
            'config/sim_base_wheels.toml',
            'config/sim_torso.toml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/mimic_sim.launch.py',
            'launch/sim_motors.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Forwards real robot joint states to Gazebo sim controllers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mimic_sim_node  = mimic_sim.mimic_sim_node:main',
            'sim_motor_node  = mimic_sim.sim_motor_node:main',
        ],
    },
)
