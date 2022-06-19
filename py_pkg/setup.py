import os
from glob import glob
from setuptools import setup

package_name = 'py_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/topic_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iztatik',
    maintainer_email='iztatik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node = py_pkg.01_node:main",
            "publisher = py_pkg.02_publisher:main",
            "subscriber = py_pkg.03_subscriber:main",
            "server = py_pkg.04_server:main",
            "client = py_pkg.05_client:main",
            "joint  = py_pkg.my_joint_publisher:main",
            "parameters = py_pkg.06_parameters:main",
            "action_server = py_pkg.07_action_server:main",
            "action_client = py_pkg.08_action_client:main"
        ],
    },
)
