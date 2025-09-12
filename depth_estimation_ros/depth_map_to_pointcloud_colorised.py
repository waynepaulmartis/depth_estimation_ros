#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import numpy as np

# (1) + (2): neue Imports für TurboJPEG RGB-Decode und Dict-Matching
from turbojpeg import TurboJPEG, TJPF_RGB, TJFLAG_FASTUPSAMPLE, TJFLAG_FASTDCT

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('four_image_subscriber')

        # (1) TurboJPEG-Instanz
        self.jpeg = TurboJPEG()

        # --- Intrinsics der KALIBRIERAUFLÖSUNG ---
        self.declare_parameter('fx_full', 894.671111)
        self.declare_parameter('fy_full', 894.671111)
        self.declare_parameter('cx_full', 967.26440338)
        self.declare_parameter('cy_full', 586.45334639)

        # Kalibrier-Auflösung
        self.declare_parameter('calib_w', 1920)
        self.declare_parameter('calib_h', 1200)

        # Optionaler Downsampling-Faktor
        self.declare_parameter('step', 4)
        self.declare_parameter('max_depth_m', 3.0)

        self.fx_full = float(self.get_parameter('fx_full').value)
        self.fy_full = float(self.get_parameter('fy_full').value)
        self.cx_full = float(self.get_parameter('cx_full').value)
        self.cy_full = float(self.get_parameter('cy_full').value)
        self.calib_w = int(self.get_parameter('calib_w').value)
        self.calib_h = int(self.get_parameter('calib_h').value)
        self.ds      = int(self.get_parameter('step').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)

        # Abos + Publisher (4 Kameras)
        self.depth_image_subs = []
        self.pc_pubs = []
        for i in range(4):
            topic = f'/depth/image_{i}'
            pc_topic = f'/pointcloud_{i}'
            self.depth_image_subs.append(
                self.create_subscription(Image, topic,
                                        lambda msg, cam=i: self.depth_cb(msg, cam), 10)
            )
            pub = self.create_publisher(PointCloud2, pc_topic, 10)
            self.pc_pubs.append(pub)
            self.get_logger().info(f"Subscribed: {topic}, Publishing: {pc_topic}")

        # (2) Compressed-Image Matching via Dict (O(1))
        # pro Kamera ein Dict: key = timestamp in ns, value = CompressedImage
        self.compressed_images = [dict() for _ in range(4)]
        # alte Einträge nach z.B. 400 ms verwerfen
        self.max_age_ns = 400_000_000

        self.compressed_image_subs = []
        for i in range(4):
            topic = f'/camera_{i}/image/compressed'
            self.compressed_image_subs.append(
                self.create_subscription(CompressedImage, topic,
                                        lambda msg, cam=i: self.compressed_image_cb(msg, cam), 10)
            )
            self.get_logger().info(f"Subscribed to compressed image topic: {topic}")

        # Caches
        self._index_cache = {}          # key: (h,w,ds) -> (uu,vv)
        self._intrinsics_cache = {}     # key: (w,h) -> (fx,fy,cx,cy)

    # (2) Hilfsfunktion: ROS2 Stamp -> int Nanoseconds
    def _stamp_to_ns(self, stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    # (1) Direkter RGB-Decode mit TurboJPEG (kein cvtColor nötig)
    def decode_with_turbo(self, jpeg_bytes):
        # TurboJPEG liefert direkt RGB (uint8)
        np_arr = self.jpeg.decode(
            jpeg_bytes,
            pixel_format=TJPF_RGB,
            flags=TJFLAG_FASTUPSAMPLE | TJFLAG_FASTDCT
        )
        return np_arr  # shape (H,W,3), dtype=uint8, RGB

    def compressed_image_cb(self, msg: CompressedImage, cam_idx: int):
        # (2) O(1) Insert in Dict + Aufräumen alter Einträge
        t = self._stamp_to_ns(msg.header.stamp)
        cam_dict = self.compressed_images[cam_idx]
        cam_dict[t] = msg

        cutoff = t - self.max_age_ns
        # einfache lineare Bereinigung (Dict typischerweise klein)
        for k in list(cam_dict.keys()):
            if k < cutoff:
                cam_dict.pop(k, None)

    def _scaled_intrinsics(self, w: int, h: int):
        key = (w, h)
        if key in self._intrinsics_cache:
            return self._intrinsics_cache[key]

        scale_w = w / float(self.calib_w)
        scale_h = h / float(self.calib_h)

        fx = self.fx_full * scale_w
        fy = self.fy_full * scale_h
        cx = self.cx_full * scale_w
        cy = self.cy_full * scale_h

        vals = (fx, fy, cx, cy)
        self._intrinsics_cache[key] = vals
        return vals

    def _get_indices(self, h: int, w: int, ds: int):
        key = (h, w, ds)
        if key in self._index_cache:
            return self._index_cache[key]

        vv, uu = np.indices((h, w), dtype=np.int32)
        if ds > 1:
            uu = uu * ds
            vv = vv * ds

        self._index_cache[key] = (uu, vv)
        return uu, vv

    def depth_cb(self, msg: Image, cam_idx: int):
        # Farbframe mit gleichem Timestamp holen
        fid = (msg.header.frame_id or "").lstrip('/')
        if fid == "cam0":
            color_data = self.find_same_timestamp_compressed_image(msg.header.stamp, 0)
        elif fid == "cam1":
            color_data = self.find_same_timestamp_compressed_image(msg.header.stamp, 1)
        elif fid == "cam2":
            color_data = self.find_same_timestamp_compressed_image(msg.header.stamp, 2)
        elif fid == "cam3":
            color_data = self.find_same_timestamp_compressed_image(msg.header.stamp, 3)
        else:
            color_data = None

        # Depth laden
        if msg.encoding in ('16UC1', 'mono16'):
            depth = np.frombuffer(msg.data, np.uint16, count=msg.height * msg.width)\
                      .reshape(msg.height, msg.width).astype(np.float32)
            depth *= 0.0005  # (0.001 / 2.0) -> schneller als zwei Divisionen
        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, np.float32, count=msg.height * msg.width)\
                      .reshape(msg.height, msg.width)
            depth *= 0.5
        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return

        in_h, in_w = depth.shape  # z. B. 300x480

        if color_data is None:

            ## Print that color is none
            self.get_logger().info(f"No Color data for frame_id: {fid}")

            # wenn ds später angewandt wird, erst mal so lassen
            pass

        if color_data is None:
            color_data = np.zeros((in_h, in_w, 3), dtype=np.uint8)

        # Intrinsics auf aktuelle Bildgröße skalieren
        fx, fy, cx, cy = self._scaled_intrinsics(in_w, in_h)

        # Optionales Downsampling der Tiefe
        if self.ds > 1:
            depth = depth[::self.ds, ::self.ds]

        h, w = depth.shape

        # Farbbild auf (h, w, 3) bringen
        if color_data is None:
            color_data = np.zeros((h, w, 3), dtype=np.uint8)
        else:
            if color_data.shape[:2] != (h, w):
                color_data = cv2.resize(color_data, (w, h), interpolation=cv2.INTER_NEAREST)

        # Indizes
        uu, vv = self._get_indices(h, w, self.ds)


        maximum_depth_m = self.max_depth_m


        z = depth
        ## Z needs to be bigger than 0 and smaller than maximum_depth_m
        z[z > maximum_depth_m] = 0.0
        mask = (z > 0.0) & np.isfinite(z)
        if not mask.any():
            return

        z_valid = z[mask]

        inv_fx = 1.0 / fx
        inv_fy = 1.0 / fy

        uu_m = uu[mask].astype(np.float32)
        vv_m = vv[mask].astype(np.float32)

        x = (uu_m - cx) * (z_valid * inv_fx)
        y = (vv_m - cy) * (z_valid * inv_fy)

        # (5) Farben extrahieren + zero-copy RGB->float Packen
        colors = color_data.reshape(-1, 3)[mask.ravel()]  # (N,3), uint8
        rgb_uint32 = (colors[:, 0].astype(np.uint32) << 16) | \
                     (colors[:, 1].astype(np.uint32) << 8)  | \
                      colors[:, 2].astype(np.uint32)
        rgb_float = rgb_uint32.view(np.float32)  # zero-copy view

        # (4) Punkte Nx4 ohne unnötige Kopien zusammenbauen
        N = z_valid.shape[0]
        pts = np.empty((N, 4), dtype=np.float32)
        pts[:, 0] = x
        pts[:, 1] = y
        pts[:, 2] = z_valid
        pts[:, 3] = rgb_float

        cloud = self._make_pc2_optical(pts, msg.header)

        # Manuelles Mapping der Frames
        if msg.header.frame_id == "/cam2":
            cloud.header.frame_id = "/rmwayne/camera00_link"
            self.pc_pubs[0].publish(cloud)
        if msg.header.frame_id == "/cam3":
            cloud.header.frame_id = "/rmwayne/camera01_link"
            self.pc_pubs[1].publish(cloud)
        if msg.header.frame_id == "/cam1":
            cloud.header.frame_id = "/rmwayne/camera02_link"
            self.pc_pubs[2].publish(cloud)
        if msg.header.frame_id == "/cam0":
            cloud.header.frame_id = "/rmwayne/camera03_link"
            self.pc_pubs[3].publish(cloud)

    # (2) O(1)-Lookup für gleichen Timestamp
    def find_same_timestamp_compressed_image(self, timestamp, cam_idx):
        t = self._stamp_to_ns(timestamp)
        img_msg = self.compressed_images[cam_idx].get(t)
        if img_msg is None:
            return None
        # (1) Direkter RGB-Decode
        return self.decode_with_turbo(img_msg.data)

    def _make_pc2_optical(self, pts: np.ndarray, header_in: Header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = header_in.stamp

        # x,y,z,rgb(float32)
        msg.height = 1
        msg.width = pts.shape[0]
        msg.is_bigendian = False
        msg.is_dense = False
        msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * pts.shape[0]
        msg.data = pts.tobytes(order='C')
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
