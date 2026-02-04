#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker

class GoalMarkerPublisher(Node):
    def __init__(self, x=1.0, y=2.0, z=0.0, frame_id='rmwayne/odom'):
        super().__init__('goal_marker_publisher')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Marker, '/goal_pose_marker', qos)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER   # <-- changed from SPHERE to CYLINDER
        marker.action = Marker.ADD

        # Position of the cylinder
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z) + 0.5  # center at half height
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size of the cylinder
        marker.scale.x = 0.08   # diameter in x
        marker.scale.y = 0.08   # diameter in y
        marker.scale.z = 0.5   # height

        # Color (green cylinder)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker = marker
        self.timer = self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerPublisher(x=6.5, y=6.0, z=0.0, frame_id='rmwayne/odom')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
