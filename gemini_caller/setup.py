from setuptools import find_packages, setup
import os
package_name = 'gemini_caller'
from glob import glob
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
           (os.path.join('share', package_name, 'templates'),
            ['gemini_caller/templates/index.html']),

        # Templates
        (os.path.join('share', package_name, 'templates'),
         glob('gemini_caller/templates/*')),

        # Static root files (if any)
        (os.path.join('share', package_name, 'static'),
         glob('gemini_caller/static/*.*')),

        # CSS files
        (os.path.join('share', package_name, 'static/css'),
         glob('gemini_caller/static/css/*.css')),

        # JS files
        (os.path.join('share', package_name, 'static/js'),
         glob('gemini_caller/static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboholic_harsh',
    maintainer_email='roboholic_harsh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gemini_caller = gemini_caller.gemini_node:main',
        ],
    },
)
