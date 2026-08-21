import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'path_planning'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'maps'
            ),
            glob('maps/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'data'
            ),
            glob('data/*')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='Daniel Del Rosario',

    maintainer_email='daniel@example.com',

    description=(
        'SLAM, LPA* global planning and B-Spline '
        'trajectory smoothing for F1TENTH AutoDRIVE.'
    ),

    license='MIT',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            (
                'waypoint_recorder = '
                'path_planning.waypoint_recorder:main'
            ),

            (
                'global_path_publisher = '
                'path_planning.global_path_publisher:main'
            ),
        ],
    },
)
