import glob
from setuptools import find_packages, setup

package_name = 'mimic_wd'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mimic_wd']),
        ('share/mimic_wd',        ['package.xml']),
        ('share/mimic_wd/launch', glob.glob('launch/*.py')),
        ('share/mimic_wd/config', glob.glob('config/*.toml')),
        ('share/mimic_wd/urdf/left_arm',  glob.glob('urdf/left_arm/*')),
        ('share/mimic_wd/urdf/right_arm', glob.glob('urdf/right_arm/*')),
        ('share/mimic_wd/urdf/robot',     glob.glob('urdf/robot/*')),
        ('share/mimic_wd/urdf/meshes',    glob.glob('urdf/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neuralzome',
    maintainer_email='tech@neuralzome.com',
    description='Mimic whole-body dynamics node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mimic_wd_node = mimic_wd.mimic_wd_node:main',
        ],
    },
)
