from setuptools import find_packages, setup
import os
from glob import glob
from pathlib import Path


package_name = 'depth_estimation_ros'


def collect_rosbags():
    root = Path('rosbags')
    if not root.exists():
        return []
    mapping = {}
    for f in root.rglob('*'):
        if f.is_file():
            rel_dir = f.parent.relative_to(root)  # Unterordner relativ
            dest = Path('share') / package_name / 'rosbags' / rel_dir
            mapping.setdefault(dest.as_posix(), []).append(f.as_posix())
    return sorted(mapping.items())

rosbags_entries = collect_rosbags()


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'Depth-Anything-V2']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz_config'), glob('rviz_config/*.rviz')),
    ]+ rosbags_entries,
    install_requires=['setuptools','numpy', 'torch', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='admin-jfinke',
    maintainer_email='wayne.paul.martis@iml.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_anything_v2_node = depth_estimation_ros.depth_anything_v2_node:main',
            'depth_estimation_onnx_node = depth_estimation_ros.depth_anything_v2_onnx_node:main',
            'depth_estimation_onnx_multi_node = depth_estimation_ros.depth_anything_v2_onnx_multi_node:main',
            'depth_map_to_pointcloud = depth_estimation_ros.depth_map_to_pointcloud:main',
            'depth_map_to_pointcloud_colorised = depth_estimation_ros.depth_map_to_pointcloud_colorised:main',
        ],
    },
)
