from setuptools import setup
import os
from glob import glob

package_name = 'miro_calibration'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='MIRO Team',
    maintainer_email='your.email@example.com',
    description='Manual calibration and hardware tuning system for MIRO v2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed_preset_manager = miro_calibration.speed_preset_manager:main',
            'feedback_monitor = miro_calibration.feedback_monitor:main',
            'sensor_health_monitor = miro_calibration.sensor_health_monitor:main',
            'data_logger = miro_calibration.data_logger:main',
        ],
    },
)
