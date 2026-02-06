from setuptools import find_packages, setup

package_name = 'automatic_pnp'

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
            'start_pnp = automatic_pnp.pnp:main',
            'clear_table = automatic_pnp.clear_table:main',
            'automated_sorting = automatic_pnp.automated_sorting:main',
            'automated_sorting2 = automatic_pnp.automated_sorting2:main',

        ],
    },
)
