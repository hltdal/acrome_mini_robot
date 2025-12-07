from setuptools import find_packages, setup

package_name = 'publish_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='halit',
    maintainer_email='dalha21@itu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_scan_from_raspbery = publish_lidar.get_scan_from_raspbery:main',
            'publish_synchronous = publish_lidar.publish_synchronous:main',
        ],
    },
)
