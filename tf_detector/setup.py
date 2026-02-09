import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tf_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboholic_harsh',
    maintainer_email='harshpjadav165@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start_vision = tf_detector.detector:main',
            'coloured_detector = tf_detector.colored_detector:main',
            'coloured_detector2 = tf_detector.colored_detector2:main',
        ],
    },
)
