from setuptools import find_packages, setup

package_name = 'depth_estimation_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'Depth-Anything-V2']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
        ],
    },
)
