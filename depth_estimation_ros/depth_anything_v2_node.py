#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import torch
import numpy as np
import time
import matplotlib.cm
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from .depth_anything_v2.depth_anything_v2.dpt import DepthAnythingV2

class DepthEstimationNode(Node):
    def __init__(self, encoder):
        super().__init__('depth_anything_v2_node')
        
        self.bridge = CvBridge()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }
        self.cmap = matplotlib.cm.get_cmap('Spectral_r')
        self.depth_model = DepthAnythingV2(**model_configs[encoder])
        checkpoint_path = f'./src/depth_estimation_ros/depth_estimation_ros/depth_anything_v2/checkpoints/depth_anything_v2_metric_lars_vits.pth'

        self.get_logger().info("Loading model checkpoint...")
        self.depth_model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'))
        self.depth_model = self.depth_model.to(self.device).eval()
        self.get_logger().info("Model checkpoint loaded successfully.")

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_0/image/compressed',
            self.image_callback,
            1)
        
        self.publisher = self.create_publisher(Image, '/depth/image', 10)
        self.get_logger().info("Depth Estimation Node Initialized")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run depth estimation
        inference_start = time.time()
        depth = self.depth_model.infer_image(cv_image, 518)  # Standard size of 518 for the model
        depth = ((depth - depth.min()) / (depth.max() - depth.min()) * 255.0).astype(np.uint8)
        depth = (self.cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
        inference_time = time.time() -inference_start
        self.get_logger().info(f"Inference Rate: {1/inference_time:.2f} FPS")

        # Convert depth image to ROS 2 Image message
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="bgr8")
        self.publisher.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)

    encoder_options = ['vits', 'vitb', 'vitl']
    print(f"Available encoders: {encoder_options}")

    encoder = input("Enter encoder type (vits/vitb/vitl): ").strip()
    while encoder not in encoder_options:
        print(f"Invalid choice! Please select from {encoder_options}.")
        encoder = input("Enter encoder type (vits/vitb/vitl): ").strip()

    node = DepthEstimationNode(encoder)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
