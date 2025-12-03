from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'humanoid_tennis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),


    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Humanoid tennis arm demo for 133a',
    license='MIT',

    entry_points={
        'console_scripts': [
            'basic_swing_node = humanoid_tennis.basic_swing_node:main',
            'simple_sim_node = humanoid_tennis.simple_sim_node:main',
        ],
    },
)
