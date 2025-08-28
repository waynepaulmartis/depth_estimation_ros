#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import torch
import onnxruntime as ort
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import matplotlib.cm


class DepthEstimationONNXNode(Node):
    def __init__(self, onnx_model_path, input_size=518):
        super().__init__('depth_estimation_onnx_node')

        self.bridge = CvBridge()
        self.input_size = input_size
        self.cmap = matplotlib.cm.get_cmap('Spectral_r')

        # Load ONNX model
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(onnx_model_path, providers=providers)
        self.device = 'cuda' if 'CUDAExecutionProvider' in self.session.get_providers() else 'cpu'
        self.get_logger().info(f'ONNX Runtime providers: {self.session.get_providers()}')
        self.get_logger().info(f'Using device: {self.device}')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_2/image/compressed',
            self.image_callback,
            1)

        self.publisher = self.create_publisher(Image, '/depth/image', 10)

        self.get_logger().info("Depth Estimation ONNX Node Initialized")

    def preprocess_image(self, cv_image):
        h, w = cv_image.shape[:2]
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) / 255.0
        image = cv2.resize(image, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        image = (image - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        image = image.transpose(2, 0, 1)[None].astype('float32')
        return image, h, w

    def run_inference(self, image):
        binding = self.session.io_binding()
        ort_input = self.session.get_inputs()[0].name
        ort_output = self.session.get_outputs()[0].name

        binding.bind_cpu_input(ort_input, image)
        binding.bind_output(ort_output, self.device)
        self.session.run_with_iobinding(binding)
        depth = binding.get_outputs()[0].numpy()
        return depth

    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {e}")
            return

        image, orig_h, orig_w = self.preprocess_image(cv_image)

        start_time = time.time()
        depth = self.run_inference(image)


        # Resize depth to original image size
        depth_tensor = torch.from_numpy(depth)
        if depth_tensor.ndim == 3:
            depth_tensor = depth_tensor.unsqueeze(0)
        depth_resized = torch.nn.functional.interpolate(
            depth_tensor,
            size=(orig_h, orig_w),
            mode='bicubic',
            align_corners=False
        ).squeeze().cpu().numpy()


        # Normalize depth for visualization
        vis_depth = (depth_resized - depth_resized.min()) / (depth_resized.max() - depth_resized.min() + 1e-8)
        vis_depth = (vis_depth * 255).astype(np.uint8)
        vis_depth_color = (self.cmap(vis_depth)[:, :, :3] * 255).astype(np.uint8)[..., ::-1]  # RGB->BGR
        inference_time = time.time() - start_time

        self.get_logger().info(f"Inference Rate: {1/inference_time:.2f} FPS")

        # Publish depth visualization
        depth_msg = self.bridge.cv2_to_imgmsg(vis_depth_color, encoding='bgr8')
        depth_msg.header = msg.header
        self.publisher.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)

    # TODO: update this to your actual ONNX model path or parameterize it
    onnx_model_path = '/home/admin-jfinke/MA_Wayne_Martis/ros2_ws/src/depth_estimation_ros/depth_estimation_ros/depth_anything_v2/checkpoints/lars_17.onnx'

    node = DepthEstimationONNXNode(onnx_model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
