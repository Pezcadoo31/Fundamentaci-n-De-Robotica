from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jess',
    maintainer_email='jess@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "signal_generator_mini = mini_challenge.signal_generator_mini:main",
          "process_mini = mini_challenge.process_mini:main",
        ],
    },
)
