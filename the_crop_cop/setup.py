import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'the_crop_cop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install URDF/XACRO models
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Install Gazebo world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Install Config YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Pest control turtle robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = the_crop_cop.patrol_node:main',
        ],
    },
)