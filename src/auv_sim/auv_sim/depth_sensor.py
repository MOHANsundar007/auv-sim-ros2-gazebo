"""Simulated pressure/depth sensor.

Fortress has no underwater pressure sensor, so depth is derived from the
ground-truth odometry with Gaussian noise.

Subscribes: /auv/odom   (nav_msgs/Odometry)
Publishes:  /auv/depth  (std_msgs/Float64, metres, positive down)
"""
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class DepthSensor(Node):

    def __init__(self):
        super().__init__('depth_sensor')
        self.noise_std = self.declare_parameter('noise_std', 0.02).value
        self.pub = self.create_publisher(Float64, '/auv/depth', 10)
        self.create_subscription(Odometry, '/auv/odom', self.on_odom, 10)

    def on_odom(self, msg):
        depth = -msg.pose.pose.position.z + random.gauss(0.0, self.noise_std)
        self.pub.publish(Float64(data=max(0.0, depth)))


def main():
    rclpy.init()
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
