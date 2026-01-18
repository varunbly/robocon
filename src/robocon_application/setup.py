from setuptools import setup
import os
from glob import glob

package_name = 'robocon_application'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='The robocon_application package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_logic_node = robocon_application.robot_logic_node:main',
            'cheetah_controller = robocon_application.cheetah_controller:main',
            'cheetah_teleop = robocon_application.cheetah_teleop:main',
            'car_controller = robocon_application.car_controller:main',
            'localization_node = robocon_application.Localization_node:main',
            'object_detection = robocon_application.ObjectDetection:main',
            'planner_node = robocon_application.Planner_node:main',
            'inverse_kinematics = robocon_application.InverseKinematics:main',
            'mapping_node = robocon_application.Maping:main',
        ],
    },
)
