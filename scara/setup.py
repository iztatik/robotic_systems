import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'scara'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name + '/meshes'), glob('meshes/*')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='invictus',
    maintainer_email='invictus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator = scara.trajectory_generator:main',
            'scara_joint_publisher = scara.scara_joint_publisher:main',
            'transparent_publisher = scara.transparent_publisher:main',
            'client = scara.client:main',
            'virtual_pendant = scara.virtual_pendant:main',
        ],
    },
)
