from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'challenge2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.[xy]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdy',
    maintainer_email='abdy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ctrl = challenge2.ctrl:main',
            'sp_gen = challenge2.sp_gen:main',
            'motor_sys = challenge2.motor_sys:main',
        ],
    },
)
