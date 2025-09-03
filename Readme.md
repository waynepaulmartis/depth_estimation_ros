
# ROS2 Package for Monocular Depth Estimation

Contains ros2 node for the DepthAnything-v2 MDE model.

## Installation

Follwing libraries are required 

```bash
pytorch
numpy
opencv-python
matplotlib
compressed_image_transport
```

## Usage

In the ros2 workspace run the following to build the package

```bash
colcon build --symlink-install
source install/setup.bash
``` 
Then run the command

```bash
ros2 run depth_estimation_ros depth_anything_v2_node
```

Visualise the depth in 

```bash
rqt
```


## File Structure

```bash
├── depth_estimation_ros/
│   ├── depth_anything_v2_node.py
│   └── __init__.py
|   └── depth_anything_v2/  # contains the DepthAnything-v2 repo
├── rosbags/
│   └── bag1.db3
├── checkpoints/
│   └── depth_anything_v2_{encoder}.pth
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

