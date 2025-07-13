from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'yopo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', ['config/traj_opt.yaml']),
        ('share/' + package_name + '/saved/YOPO_1/', ['saved/YOPO_1/epoch50.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='legion',
    maintainer_email='3394352059@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yopo_node = yopo.test_yopo_ros2:main',
            'train_yopo = yopo.train_yopo:main',
        ],
    },
)
