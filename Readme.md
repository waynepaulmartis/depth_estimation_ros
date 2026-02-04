
# ROS2 Package for Monocular Depth Estimation

Contains ros2 node for the DepthAnything-v2 MDE model. This repo is built using the following references

- [Depth-Anything-ONNX](https://github.com/fabio-sim/Depth-Anything-ONNX)
- [Depth-Anything-v2](https://github.com/DepthAnything/Depth-Anything-V2)


## File Structure

```bash
├── depth_estimation_ros/           # ROS2 package directory
│   ├── __init__.py
│   ├── depth_anything_v2_node.py
│   ├── depth_anything_v2_onnx_node.py
│   ├── depth_anything_v2_onnx_multi_node.py
│   ├── depth_map_to_pointcloud.py
│   ├── depth_map_to_pointcloud_colorised.py
│   ├── goal_publisher.py
│   ├── depth_anything_v2/          # DepthAnything-v2 repository for reference
├── launch/
│   ├── depth_estimation_and_pointcloud.launch.py
│   ├── depth_map_to_pointcloud_node.launch.py
│   ├── depth_map_to_pointcloud_colorised_node.launch.py
│   └── play_mde_rl_rosbag.launch.py
├── checkpoints/                    # ONNX model checkpoints
├── package.xml
├── setup.py
└── Readme.md
```

## Installation

Create and activate a conda environment with the required dependencies:

```bash
# Create conda environment
conda create -n depth_estimation python=3.10 -y

# Activate the environment
conda activate depth_estimation

# Install required libraries
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
pip install numpy opencv-python matplotlib

# Install ROS2 compressed image transport (system dependency)
sudo apt-get install ros-${ROS_DISTRO}-compressed-image-transport
```

## Checkpoints

The ONNX checkpoints are available in the /checkpoints folder

## Usage

In the ros2 workspace run the following to build the package

```bash
colcon build --symlink-install
source install/setup.bash
``` 
Then run the command

```bash
ros2 launch depth_estimation_ros depth_estimation_and_pointcloud.launch.py
```

### Visualisation
```bash
rqt
```


## Citation

Please cite

```bash
@article{depth_anything_v2,
  title={Depth Anything V2},
  author={Yang, Lihe and Kang, Bingyi and Huang, Zilong and Zhao, Zhen and Xu, Xiaogang and Feng, Jiashi and Zhao, Hengshuang},
  journal={arXiv:2406.09414},
  year={2024}
}
```

