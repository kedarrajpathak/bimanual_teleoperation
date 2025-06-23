from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        '': ['config/*.yaml', 'config/*.npy', 'launch/*.py', 'hand_tracking/*.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.npy')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'aruco_pose_estimation'), glob('aruco_pose_estimation/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kedar Rajpathak',
    maintainer_email='kedar.rajpathak@rwth-aachen.de',
    description='Package to track and estimate poses of aruco markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_pose_estimation_node = aruco_pose_estimation.aruco_pose_estimation_node:main',
        ],
    },
)
