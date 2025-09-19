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
import message_filters
from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY, TJFLAG_PROGRESSIVE, TJFLAG_FASTUPSAMPLE, TJFLAG_FASTDCT


class DepthEstimationONNXMultiNode(Node):
    def __init__(self, onnx_model_path, input_size=518):
        super().__init__('depth_estimation_multi_onnx_node')

        self.bridge = CvBridge()
        self.input_size = input_size
        self.cmap = matplotlib.cm.get_cmap('Spectral_r')

        # Load ONNX model
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(onnx_model_path, providers=providers)
        self.device = 'cuda' if 'CUDAExecutionProvider' in self.session.get_providers() else 'cpu'
        self.get_logger().info(f'ONNX Runtime providers: {self.session.get_providers()}')
        self.get_logger().info(f'Using device: {self.device}')

        # Topics
        self.topic_names = [f'/camera_{i}/image/compressed' for i in range(4)]

        # Depth publishers
        self.depth_publishers = [
            self.create_publisher(Image, f'/depth/image_{i}', 10) for i in range(4)
        ]

        # Subscribers with synchronization
        subs = [message_filters.Subscriber(self, CompressedImage, topic) 
                for topic in self.topic_names]

        # Approximate sync: allows small timestamp mismatch
        self.ts = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=5, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.jpeg = TurboJPEG()
        self.get_logger().info("Depth Estimation ONNX Node Initialized with synchronized subscribers")

    def preprocess_image(self, cv_image):
        h, w = cv_image.shape[:2]
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) / 255.0
        image = cv2.resize(image, (self.input_size, self.input_size), interpolation=cv2.INTER_CUBIC)
        image = (image - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        image = image.transpose(2, 0, 1)[None].astype('float32')
        return image, h, w

    def run_inference(self, batch_input):
        binding = self.session.io_binding()
        ort_input = self.session.get_inputs()[0].name
        ort_output = self.session.get_outputs()[0].name

        binding.bind_cpu_input(ort_input, batch_input)
        binding.bind_output(ort_output, self.device)
        self.session.run_with_iobinding(binding)
        depth = binding.get_outputs()[0].numpy()
        return depth

    def sync_callback(self, img0: CompressedImage, img1: CompressedImage,
                      img2: CompressedImage, img3: CompressedImage):
        images = [img0, img1, img2, img3]

        # Compute average timestamp for latency
        avg_timestamp_msg = np.mean([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in images])
        timestamp_system = time.time()
        latency = (timestamp_system - avg_timestamp_msg) * 1000  # ms
        self.get_logger().info(f"Avg sync latency: {latency:.2f} ms")

        cv_images = []
        for idx, msg in enumerate(images):
            try:
                t0 = time.time()
                cv_image = self.jpeg.decode(msg.data, flags=TJFLAG_FASTUPSAMPLE | TJFLAG_FASTDCT)
                t1 = time.time()
                self.get_logger().info(f"Time to decode camera {idx}: {(t1-t0)*1000:.2f} ms")
                cv_images.append((cv_image, msg.header))
            except Exception as e:
                self.get_logger().error(f"Failed to decode image {idx}: {e}")
                return

        # Now process all 4 images together
        self.process_batch(cv_images)

        final_timestamp = time.time()
        total_latency = (final_timestamp - avg_timestamp_msg) * 1000
        self.get_logger().info(f"Total sync latency: {total_latency:.2f} ms")


    def process_batch(self, cv_images):
        batch_input = np.zeros((4, 3, self.input_size, self.input_size), dtype=np.float32)
        headers, orig_sizes = [], []

        for i, (cv_image, header) in enumerate(cv_images):
            img, h, w = self.preprocess_image(cv_image)
            batch_input[i] = img
            headers.append(header)
            orig_sizes.append((h, w))

        # Run inference
        start_time = time.time()
        depth_batch = self.run_inference(batch_input)
        inference_time = time.time() - start_time
        self.get_logger().info(
            f"Batched inference: {inference_time*1000:.2f} ms "
            f"({1/inference_time:.2f} FPS)"
        )

        # Resize and publish results
        for i in range(4):
            h, w = orig_sizes[i]
            depth_resized = cv2.resize(
                depth_batch[i].astype(np.float32),
                (w, h),
                interpolation=cv2.INTER_CUBIC
            )

            depth_msg = self.bridge.cv2_to_imgmsg(depth_resized, encoding='32FC1')
            depth_msg.header = headers[i]
            self.depth_publishers[i].publish(depth_msg)



def main(args=None):
    rclpy.init(args=args)

    onnx_model_path = '/home/admin-jfinke/MA_Wayne_Martis/ros2_ws/src/depth_estimation_ros/depth_estimation_ros/depth_anything_v2/checkpoints/lars_multi.onnx'

    node = DepthEstimationONNXMultiNode(onnx_model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
