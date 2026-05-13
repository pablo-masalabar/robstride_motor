import glob
from setuptools import find_packages, setup

package_name = 'mimic'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mimic']),
        ('share/mimic',        ['package.xml']),
        ('share/mimic/launch', glob.glob('launch/*')),
        ('share/mimic/config', glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Mimic node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mimic_node = mimic.mimic_node:main',
        ],
    },
)
