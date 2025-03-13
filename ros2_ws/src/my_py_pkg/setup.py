from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_py_pkg'

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
    maintainer='abdy',
    maintainer_email='abdy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"hola_mundo = my_py_pkg.node_python_1:main",
        	"hola_incrementar = my_py_pkg.node_python_t:main",
        	"my_talker = my_py_pkg.my_talker:main",
        	"my_listener = my_py_pkg.my_listener:main",
            "add_2ints_server = my_py_pkg.add2ints_server:main",
            "add_2ints_client = my_py_pkg.add2ints_client:main",
            "hw_status_publisher = my_py_pkg.HardwareStatus_publisher:main",
            "hw_status_subscriber = my_py_pkg.HardwareStatus_subscriber:main",
            "cra_client = my_py_pkg.ComputeRectangleArea_client:main",
            "cra_server = my_py_pkg.ComputeRectangleArea_server:main",
        ],
    },
)
