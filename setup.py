from glob import glob
from setuptools import find_packages, setup

setup(
    name='match_mir_camera_calibration', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/match_mir_camera_calibration']),
        ('share/match_mir_camera_calibration', ['package.xml', 'README.md', 'LICENSE']),
        ('share/match_mir_camera_calibration/config', glob('config/*.yaml')),
    ],
    package_data={'match_mir_camera_calibration': ['web/*.html']},
    install_requires=['setuptools'], zip_safe=False,
    maintainer='MATCH', maintainer_email='rosmatch@example.com', license='MIT',
    description='Mocap-assisted joint MiR camera and ArUco marker calibration',
    entry_points={'console_scripts': [
        'mir_camera_calibration_gui = match_mir_camera_calibration.gui:main',
        'calibration_session = match_mir_camera_calibration.session_node:main',
        'calibration_web = match_mir_camera_calibration.web:main',
        'calibrate_session = match_mir_camera_calibration.calibration:main',
    ]},
)
