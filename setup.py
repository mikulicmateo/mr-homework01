from setuptools import setup
import os
from glob import glob

package_name = 'mmikulic_1'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('rviz/*.[rviz]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateo Mikulic',
    maintainer_email='mmikulic@riteh.hr',
    description='1. Domaca zadaca',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_subscriber = mmikulic_1.position_subscriber:main',
            'position_controller = mmikulic_1.position_controller:main'
        ],
    },
)
