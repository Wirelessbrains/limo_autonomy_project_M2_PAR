import os
from glob import glob
from setuptools import setup

package_name = 'limo_online_relocalization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joao-pedro',
    maintainer_email='joao_pedro.martins_do_lago_reis@etu.uca.fr',
    description='Online relocalization against a recorded reference trajectory with RViz visualization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'online_relocalization_node = limo_online_relocalization.online_relocalization_node:main',
        ],
    },
)
