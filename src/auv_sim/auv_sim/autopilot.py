"""Depth / heading / thrust autopilot (cascaded PID) for the AUV.

  depth error --PID--> desired pitch --PD--> elevator
  heading error ----------PD--------------> rudder
  thrust: open loop

Sign conventions (ENU, verified in sim):
  pitch > 0 = nose down,  elevator < 0 = nose down (dive)
  rudder > 0 = turn right (yaw decreases)

Setpoints (change at runtime):
  /auv/setpoint/depth    std_msgs/Float64  [m, positive down]
  /auv/setpoint/heading  std_msgs/Float64  [deg, ENU: 0 = +x, 90 = +y]
  /auv/setpoint/thrust   std_msgs/Float64  [N]
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

FIN_MAX = 0.35


def clamp(v, lim):
    return max(-lim, min(lim, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(clamp(2 * (q.w * q.y - q.z * q.x), 1.0))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class Autopilot(Node):

    def __init__(self):
        super().__init__('auv_autopilot')
        p = self.declare_parameter
        self.depth_sp = p('target_depth', 5.0).value
        self.heading_sp = math.radians(p('target_heading_deg', 0.0).value)
        self.thrust_sp = p('thrust', 20.0).value
        self.kp_depth = p('kp_depth', 0.2).value
        self.ki_depth = p('ki_depth', 0.01).value
        self.kd_depth = p('kd_depth', 0.2).value
        self.max_pitch = p('max_pitch', 0.5).value
        self.kp_pitch = p('kp_pitch', 1.0).value
        self.kd_pitch = p('kd_pitch', 0.5).value
        self.kp_heading = p('kp_heading', 1.0).value
        self.kd_heading = p('kd_heading', 0.6).value

        self.depth_int = 0.0
        self.depth_rate = 0.0
        self.last_depth = None
        self.last_t = None

        self.thrust_pub = self.create_publisher(Float64, '/auv/thrust', 10)
        self.rudder_pub = self.create_publisher(Float64, '/auv/rudder', 10)
        self.elevator_pub = self.create_publisher(Float64, '/auv/elevator', 10)

        self.create_subscription(Odometry, '/auv/odom', self.on_odom, 10)
        self.create_subscription(Float64, '/auv/setpoint/depth', self.on_depth_sp, 10)
        self.create_subscription(Float64, '/auv/setpoint/heading', self.on_heading_sp, 10)
        self.create_subscription(Float64, '/auv/setpoint/thrust', self.on_thrust_sp, 10)

        self.get_logger().info(
            f'Autopilot: depth={self.depth_sp:.1f} m, '
            f'heading={math.degrees(self.heading_sp):.0f} deg, thrust={self.thrust_sp:.0f} N')

    def on_depth_sp(self, msg):
        self.depth_sp = msg.data
        self.get_logger().info(f'depth setpoint -> {msg.data:.1f} m')

    def on_heading_sp(self, msg):
        self.heading_sp = math.radians(msg.data)
        self.get_logger().info(f'heading setpoint -> {msg.data:.0f} deg')

    def on_thrust_sp(self, msg):
        self.thrust_sp = msg.data
        self.get_logger().info(f'thrust setpoint -> {msg.data:.0f} N')

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t

        _, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
        q = msg.twist.twist.angular.y   # body pitch rate
        r = msg.twist.twist.angular.z   # body yaw rate
        depth = -msg.pose.pose.position.z

        # Depth rate (filtered finite difference), +ve = descending
        if self.last_depth is not None and dt > 0.0:
            self.depth_rate += 0.2 * ((depth - self.last_depth) / dt - self.depth_rate)
        self.last_depth = depth

        # Depth -> desired pitch (PID). Integrate only near the setpoint so
        # large depth changes don't wind up the integrator.
        depth_err = self.depth_sp - depth
        if abs(depth_err) < 1.0:
            self.depth_int = clamp(self.depth_int + depth_err * dt,
                                   0.3 / max(self.ki_depth, 1e-6))
        pitch_sp = clamp(self.kp_depth * depth_err + self.ki_depth * self.depth_int
                         - self.kd_depth * self.depth_rate, self.max_pitch)

        # Pitch -> elevator (PD); -ve elevator pitches nose down
        elevator = -(self.kp_pitch * (pitch_sp - pitch) - self.kd_pitch * q)

        # Heading -> rudder (PD); +ve rudder decreases yaw
        rudder = -(self.kp_heading * wrap(self.heading_sp - yaw) - self.kd_heading * r)

        self.thrust_pub.publish(Float64(data=self.thrust_sp))
        self.elevator_pub.publish(Float64(data=clamp(elevator, FIN_MAX)))
        self.rudder_pub.publish(Float64(data=clamp(rudder, FIN_MAX)))

    def stop(self):
        for pub in (self.thrust_pub, self.rudder_pub, self.elevator_pub):
            pub.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = Autopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
