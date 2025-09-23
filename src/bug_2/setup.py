from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bug_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # adiciona launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bug2main = bug_2.bug_2_main:main',
        ],
    },
)
