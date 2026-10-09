import os
from glob import glob
from setuptools import setup

package_name = 'identify_pack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.vision_identify', package_name + '.serial', package_name + '.scripts', package_name + '.tf_transform'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('identify_pack/serial/serial_config.json')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/color_config.json')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/ROI_Range.txt')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/Arrival_ROI.txt')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/turntable_detector.json')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/turntable_material_seed.json')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/ring_station_detector.json')),
        (os.path.join('share', package_name), glob('identify_pack/scripts/stacked_ring_detector.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lulu',
    maintainer_email='lulu@todo.todo',
    description='GCS QR decode and serial bridge nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pixel_to_base = identify_pack.tf_transform.pixel_to_base_node:main',
            'place_task = identify_pack.vision_identify.place_task_node:main',
            'tf_transform = identify_pack.tf_transform.tf_transform_node:main',
            'camera_node = identify_pack.vision_identify.camera_node:main',
            'circle_detect_node = identify_pack.vision_identify.circle_detect_node:main',
            'color_detect_node = identify_pack.vision_identify.color_detect_node:main',
            'gripper_arrival = identify_pack.vision_identify.gripper_arrival_node:main',
            'turntable_detect_node = identify_pack.vision_identify.turntable_detect_node:main',
            'ring_station_detect_node = identify_pack.vision_identify.ring_station_detect_node:main',
            'stacked_ring_detect_node = identify_pack.vision_identify.stacked_ring_detect_node:main',
            'qr_decode_node = identify_pack.vision_identify.qr_decode_node:main',
            'qr_serial_bridge = identify_pack.serial.qr_serial_bridge:main',
            'serial_bridge_node = identify_pack.serial.serial_bridge_node:main',
            'roi_overlay_node = identify_pack.scripts.roi_overlay_node:main',
            'color_calibrator = identify_pack.scripts.color_calibrator:main',
            'turntable_calibrator = identify_pack.scripts.turntable_calibrator:main',
        ],
    },
)
