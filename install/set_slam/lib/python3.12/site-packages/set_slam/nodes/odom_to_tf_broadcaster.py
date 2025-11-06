#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros

class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info(f"Odom->TF broadcaster started, subscribing to: {odom_topic}")

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # expecting "odom"
        # child_frame_id typically "base_link"
        t.child_frame_id = msg.child_frame_id if msg.child_frame_id else "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
