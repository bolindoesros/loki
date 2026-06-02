#!/usr/bin/env python3
"""
converts odometry messages to paths for visualization in foxglove
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.ekf_path = Path()
        self.gt_path  = Path()
        self.ekf_path.header.frame_id = 'odom'
        self.gt_path.header.frame_id  = 'odom'

        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_cb,  10)

        self.ekf_pub = self.create_publisher(Path, '/path/ekf',          10)
        self.gt_pub  = self.create_publisher(Path, '/path/ground_truth', 10)

    def ekf_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose
        self.ekf_path.header.stamp = msg.header.stamp
        self.ekf_path.poses.append(pose)
        self.ekf_pub.publish(self.ekf_path)

    def gt_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose
        self.gt_path.header.stamp = msg.header.stamp
        self.gt_path.poses.append(pose)
        self.gt_pub.publish(self.gt_path)

def main():
    rclpy.init()
    rclpy.spin(PathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()