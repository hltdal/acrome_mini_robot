from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'set_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['set_slam', 'set_slam.*']),  # 🔧 DEĞİŞTİ
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('set_slam/launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='halit',
    maintainer_email='halit@example.com',
    description='SLAM setup using slam_toolbox with odom and scan topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf_broadcaster = set_slam.nodes.odom_to_tf_broadcaster:main',
        ],
    },
)
