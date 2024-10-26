#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from turtlesim.msg import Pose
import tf2_ros
import math

class TurtlesimTfBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'turtle1_base'
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0
        quaternion = self.quaternion_from_euler(0, 0, pose.theta)
        t.transform.rotation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        self.broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    broadcaster = TurtlesimTfBroadcaster()
    rclpy.spin(broadcaster)
    broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

