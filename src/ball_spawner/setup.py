from setuptools import setup
import os
from glob import glob


package_name = 'ball_spawner'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Ball spawner for RViz / tracking demo',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ball_spawner_node = ball_spawner.ball_spawner_node:main',
        ],
    },
)





