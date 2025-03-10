from setuptools import find_packages, setup
import os
from glob import glob
package_name = '516_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyquaternion'],
    zip_safe=True,
    maintainer='jason',
    maintainer_email='jason@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen3lite_pymoveit2 = 516_final.gen3lite_pymoveit2:main',
            'pick_place_move = 516_final.pick_place_move:main',
            ],
    },
)
