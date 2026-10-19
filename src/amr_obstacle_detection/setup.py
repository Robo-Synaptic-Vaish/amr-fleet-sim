from setuptools import find_packages, setup

package_name = 'amr_obstacle_detection'

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
    maintainer='Vaishnavi',
    maintainer_email='vaishnavi_muse@todo.todo',
    description='LiDAR-based obstacle detection and safe stopping for AMR',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector = amr_obstacle_detection.obstacle_detector:main',
        ],
    },
)
