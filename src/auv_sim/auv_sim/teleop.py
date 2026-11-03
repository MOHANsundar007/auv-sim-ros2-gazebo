"""Keyboard teleop for the AUV.

Run in its own terminal:  ros2 run auv_sim teleop
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

HELP = """
AUV keyboard teleop
-------------------
  w / s : thrust  +/- 5 N      (max 60 N)
  a / d : rudder  left / right
  i / k : elevator dive / climb
  c     : centre fins
  space : stop (zero thrust, centre fins)
  q     : quit
"""

THRUST_STEP = 5.0
THRUST_MAX = 60.0
FIN_STEP = 0.05
FIN_MAX = 0.35


def clamp(v, lim):
    return max(-lim, min(lim, v))


class Teleop(Node):

    def __init__(self):
        super().__init__('auv_teleop')
        self.thrust_pub = self.create_publisher(Float64, '/auv/thrust', 10)
        self.rudder_pub = self.create_publisher(Float64, '/auv/rudder', 10)
        self.elevator_pub = self.create_publisher(Float64, '/auv/elevator', 10)
        self.thrust = self.rudder = self.elevator = 0.0

    def on_key(self, key):
        if key == 'w':
            self.thrust += THRUST_STEP
        elif key == 's':
            self.thrust -= THRUST_STEP
        elif key == 'a':
            self.rudder -= FIN_STEP      # -ve rudder turns left
        elif key == 'd':
            self.rudder += FIN_STEP
        elif key == 'i':
            self.elevator -= FIN_STEP    # -ve elevator dives
        elif key == 'k':
            self.elevator += FIN_STEP
        elif key == 'c':
            self.rudder = self.elevator = 0.0
        elif key == ' ':
            self.thrust = self.rudder = self.elevator = 0.0
        self.thrust = clamp(self.thrust, THRUST_MAX)
        self.rudder = clamp(self.rudder, FIN_MAX)
        self.elevator = clamp(self.elevator, FIN_MAX)

    def publish(self):
        self.thrust_pub.publish(Float64(data=self.thrust))
        self.rudder_pub.publish(Float64(data=self.rudder))
        self.elevator_pub.publish(Float64(data=self.elevator))


def main():
    rclpy.init()
    node = Teleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    break
                node.on_key(key)
                print(f'\rthrust={node.thrust:5.1f} N  rudder={node.rudder:+.2f}  '
                      f'elevator={node.elevator:+.2f}   ', end='', flush=True)
            node.publish()
    finally:
        node.thrust = node.rudder = node.elevator = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()
        print()
