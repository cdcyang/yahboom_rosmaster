#!/usr/bin/env python3

from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


class PoseStampedGenerator(Node):

    def __init__(self, node_name='pose_stamped_generator'):
        super().__init__(node_name)

    def create_pose_stamped(self, x=0.0, y=0.0, z=0.0,
                            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                            frame_id='map'):
        pose_stamped = PoseStamped()
        header = Header()
        header.frame_id = frame_id

        now = self.get_clock().now()
        header.stamp = Time(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1]
        )

        position = Point()
        position.x = float(x)
        position.y = float(y)
        position.z = float(z)

        orientation = Quaternion()
        orientation.x = float(qx)
        orientation.y = float(qy)
        orientation.z = float(qz)
        orientation.w = float(qw)

        pose = Pose()
        pose.position = position
        pose.orientation = orientation
        pose_stamped.header = header
        pose_stamped.pose = pose
        return pose_stamped
