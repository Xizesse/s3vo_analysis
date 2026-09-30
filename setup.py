import os
from glob import glob

from setuptools import find_packages, setup

package_name = 's3vo_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Pires da Silva',
    maintainer_email='kikopiressilva@gmail.com',
    description='Analysis tools and bag-patching nodes for the S3VO bags.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = s3vo_analysis.odom_tf_broadcaster:main',
            'gps_origin_tf = s3vo_analysis.gps_origin_tf:main',
            'odom_reliable_relay = s3vo_analysis.odom_reliable_relay:main',
            'goal_marker_publisher = s3vo_analysis.goal_marker_publisher:main',
            'odom_heading_publisher = s3vo_analysis.odom_heading_publisher:main',
        ],
    },
)
