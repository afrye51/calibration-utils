from setuptools import setup
import os
from glob import glob

package_name = 'extrinsic_calibration_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Annika Frye',
    maintainer_email='annika@frenchfryes.com',
    description='Tools and visualizations for extrinsic sensor calibration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_lidar_record_live = scripts.cam_lidar_calib_record_live:main',
            'tf2_tweaker = scripts.tf2_tweaker:main'
        ],
    },
)
