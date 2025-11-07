import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'map_based'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**.py')),
    ],
    install_requires=['setuptools', 'PySDL2'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_based_main = map_based.map_based_main:main',
            'calculate_rrt = map_based.scripts.draw_rrt:main',
            'calculate_potential_fields = map_based.scripts.draw_potential_fields:main',
        ],
    },
)
