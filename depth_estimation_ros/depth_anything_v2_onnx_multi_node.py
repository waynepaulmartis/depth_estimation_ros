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

        # Setup subscriptions and publishers
        self.topic_names = [f'/camera_{i}/image/compressed' for i in range(4)]
        self.image_buffers = {i: None for i in range(4)}
        self.subscribers = []
        self.depth_publishers = []

        for i, topic in enumerate(self.topic_names):
            sub = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, idx=i: self.image_callback(msg, idx),
                1
            )
            self.subscribers.append(sub)

            pub = self.create_publisher(Image, f'/depth/image_{i}', 10)
            self.depth_publishers.append(pub)

        self.get_logger().info("Depth Estimation ONNX Node Initialized")

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

    def image_callback(self, msg: CompressedImage, cam_idx: int):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)

            timestamp_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamp_system = time.time()
            latency = (timestamp_system - timestamp_msg) * 1000  # in milliseconds  
            self.get_logger().info(f"Latency of image from camera {cam_idx}: {latency:.2f} ms")
            ## print latency of image
        

            ## Print time it took to decode
            jpeg = TurboJPEG()

            t0=time.time()
            #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = jpeg.decode(msg.data,flags=TJFLAG_FASTUPSAMPLE|TJFLAG_FASTDCT)
            t1=time.time()
            self.get_logger().info(f"Time to decode image from camera {cam_idx}: {(t1-t0)*1000:.2f} ms")
            ## Decode with turbo jpeg 
            #jpeg = TurboJPEG()
            #cv_image = jpeg.decode(msg.data, pixel_format=TJPF_GRAY, flags=TJFLAG_FASTUPSAMPLE | TJFLAG_FASTDCT)

        except Exception as e:
            self.get_logger().error(f"Failed to decode image from camera {cam_idx}: {e}")
            return

        self.image_buffers[cam_idx] = (cv_image, msg.header)

        if all(self.image_buffers.values()):
            self.process_batch()
            self.image_buffers = {i: None for i in range(4)}

    def process_batch(self):
        images = []
        headers = []
        orig_sizes = []

        for i in range(4):
            cv_image, header = self.image_buffers[i]
            img, h, w = self.preprocess_image(cv_image)
            images.append(img)
            headers.append(header)
            orig_sizes.append((h, w))

        batch_input = np.concatenate(images, axis=0)

        start_time = time.time()
        depth_batch = self.run_inference(batch_input)
        inference_time = time.time() - start_time
        self.get_logger().info(f"Batched Inference Rate: {1/inference_time:.2f} FPS")

        for i in range(4):
            depth = depth_batch[i]

            h, w = orig_sizes[i]

            depth_tensor = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0)
            depth_resized = torch.nn.functional.interpolate(
                depth_tensor,
                size=(h, w),
                mode='bicubic',
                align_corners=False
            ).squeeze().cpu().numpy()

            t0 = time.time()
            # Publish the resized depth map as a 32FC1 image
            depth_msg = self.bridge.cv2_to_imgmsg(depth_resized.astype(np.float32), encoding='32FC1')
            t1 = time.time()
            self.get_logger().info(f"Time to process depth image from camera {i}: {(t1-t0)*1000:.2f} ms")
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
