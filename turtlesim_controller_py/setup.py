from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlesim_controller_py'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Toshiki Nakamura',
    maintainer_email='tnakamura.amsl@gmail.com',
    description='The turtlesim_controller_py package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_controller_node_py = turtlesim_controller_py.turtlesim_controller_node:main'
        ],
    },
)
