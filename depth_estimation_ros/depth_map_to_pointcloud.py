#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('four_image_subscriber')

        # --- Intrinsics der KALIBRIERAUFLÖSUNG (z. B. 1920x1200) ---
        self.declare_parameter('fx_full', 894.671111)
        self.declare_parameter('fy_full', 894.671111)
        self.declare_parameter('cx_full', 967.26440338)
        self.declare_parameter('cy_full', 586.45334639)

        # Kalibrier-Auflösung (zum Rescaling auf das empfangene Bild)
        self.declare_parameter('calib_w', 1920)
        self.declare_parameter('calib_h', 1200)

        # Optionaler Downsampling-Faktor (auf dem empfangenen Depth-Bild)
        self.declare_parameter('step', 4)

        self.fx_full = float(self.get_parameter('fx_full').value)
        self.fy_full = float(self.get_parameter('fy_full').value)
        self.cx_full = float(self.get_parameter('cx_full').value)
        self.cy_full = float(self.get_parameter('cy_full').value)
        self.calib_w = int(self.get_parameter('calib_w').value)
        self.calib_h = int(self.get_parameter('calib_h').value)
        self.ds      = int(self.get_parameter('step').value)

        # Abos + Publisher (4 Kameras)
        self.subs = []
        self.pc_pubs = []
        for i in range(4):
            topic = f'/depth/image_{i}'
            pc_topic = f'/pointcloud_{i}'
            self.subs.append(
                self.create_subscription(Image, topic,
                                        lambda msg, cam=i: self.depth_cb(msg, cam), 10)
            )
            pub = self.create_publisher(PointCloud2, pc_topic, 10)
            self.pc_pubs.append(pub)
            self.get_logger().info(f"Subscribed: {topic}, Publishing: {pc_topic}")

        # Caches für Performance
        self._index_cache = {}          # key: (h,w,ds) -> (uu,vv)
        self._intrinsics_cache = {}     # key: (w,h) -> (fx,fy,cx,cy)

    # ---------- Hilfsfunktionen ----------
    def _scaled_intrinsics(self, w: int, h: int):
        """
        Skaliert Intrinsics von der Kalibrierauflösung (calib_w/calib_h)
        auf die *aktuelle* Bildgröße (w/h). Gleiches FOV vorausgesetzt.
        """
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
        """
        Liefert (uu,vv) für das ggf. gedownsamplte Bild; Pixelkoordinaten
        werden zurück auf die *Eingangsbild*-Skala hochskaliert, damit sie
        zu den (ebenfalls skalierten) Intrinsics passen.
        """
        key = (h, w, ds)
        if key in self._index_cache:
            return self._index_cache[key]

        vv, uu = np.indices((h, w), dtype=np.int32)
        if ds > 1:
            uu = uu * ds
            vv = vv * ds

        self._index_cache[key] = (uu, vv)
        return uu, vv

    # ---------- Haupt-Callback ----------
    def depth_cb(self, msg: Image, cam_idx: int):
        # Depth laden
        if msg.encoding in ('16UC1', 'mono16'):
            depth = np.frombuffer(msg.data, np.uint16, count=msg.height * msg.width)\
                      .reshape(msg.height, msg.width).astype(np.float32) * 0.001
            ## Divide Depth by 2
            depth = depth / 2.0

        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, np.float32, count=msg.height * msg.width)\
                      .reshape(msg.height, msg.width)
            ## Divide Depth by 2
            depth = depth / 2.0

        else:
            self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
            return

        in_h, in_w = depth.shape  # z. B. 300x480

        # Intrinsics auf aktuelle Bildgröße (z. B. 480x300) skalieren
        fx, fy, cx, cy = self._scaled_intrinsics(in_w, in_h)

        # Optionales Downsampling
        if self.ds > 1:
            depth = depth[::self.ds, ::self.ds]

        h, w = depth.shape

        # Indizes erzeugen und auf Eingangsbild-Skala zurückskalieren
        uu, vv = self._get_indices(h, w, self.ds)

        z = depth
        mask = (z > 0.0) & np.isfinite(z)
        if not mask.any():
            return

        # Projektion in *optischen* Kamerakoordinaten:
        # x: rechts, y: runter (nach unten), z: vorwärts
        z_valid = z[mask]
        inv_fx = 1.0 / fx
        inv_fy = 1.0 / fy

        uu_m = uu[mask].astype(np.float32)
        vv_m = vv[mask].astype(np.float32)

        x = (uu_m - cx) * (z_valid * inv_fx)
        y = (vv_m - cy) * (z_valid * inv_fy)
        # z = z_valid (bereits)

        # Punkte stapeln (optischer Frame → direkt so publizieren)
        pts = np.column_stack((x, y, z_valid)).astype(np.float32, copy=False)

        cloud = self._make_pc2_optical(pts, msg.header)



        ## Apply Some Manual Mapping here to match the URDF Frames
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
        
        #self.pc_pubs[cam_idx].publish(cloud)

    def _make_pc2_optical(self, pts: np.ndarray, header_in: Header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = header_in.stamp

        # Im *optischen* Frame veröffentlichen (damit Z vorwärts & Y nach unten ist)
        fid = (header_in.frame_id or "").lstrip('/')

        #if fid == "cam0":
        #    msg.header.frame_id = "rmwayne/camera00_link"
        #elif fid == "cam1":
        #    msg.header.frame_id = "rmwayne/camera01_link"
        #elif fid == "cam2":
        #    msg.header.frame_id = "rmwayne/camera02_link"
        #elif fid == "cam3":
        #    msg.header.frame_id = "rmwayne/camera03_link"
        #else:
            # Fallback (falls dein Sensor andere Frame-IDs liefert)
        #    msg.header.frame_id = "camera_optical_frame"

        msg.height = 1
        msg.width = pts.shape[0]
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        # direkt als Bytes (C-Order) einfüllen
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
