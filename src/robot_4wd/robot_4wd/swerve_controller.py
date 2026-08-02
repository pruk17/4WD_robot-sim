#!/usr/bin/env python3
"""
Swerve-drive inverse kinematics node.

Subscribes:  /cmd_vel  (geometry_msgs/Twist)
    linear.x  -> vx  (forward,  m/s)
    linear.y  -> vy  (left,     m/s)   <-- holonomic! diff-drive can't do this
    angular.z -> w   (yaw,      rad/s)

Publishes (std_msgs/Float64), one pair per corner module:
    <p>_steer_cmd  -> desired steering angle  [rad]  (position controller)
    <p>_drive_cmd  -> desired wheel speed     [rad/s](velocity controller)

For a body twist (vx, vy, w), the ground velocity of the wheel at (x, y) is
    vxi = vx - w * y
    vyi = vy + w * x
    angle = atan2(vyi, vxi)          # where to point the wheel
    speed = hypot(vxi, vyi)          # how fast to roll (m/s)
    wheel_omega = speed / r          # -> rad/s
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Steer-axis positions relative to base_link center [m] (x forward, y left).
# = the wheel COLUMN (cx/cy) in description/robot_core_mesh.xacro.
MODULES = {
    'fl': (0.26, 0.26),
    'fr': (0.26, -0.26),
    'bl': (-0.26, 0.26),
    'br': (-0.26, -0.26),
}

WHEEL_RADIUS = 0.064  # 5 inch wheel radius [m]


def wrap(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        self.steer_pub = {}
        self.drive_pub = {}
        for k in MODULES:
            self.steer_pub[k] = self.create_publisher(Float64, f'{k}_steer_cmd', 10)
            self.drive_pub[k] = self.create_publisher(Float64, f'{k}_drive_cmd', 10)

        # remember last commanded steering angle for "shortest turn" optimisation
        self.last_angle = {k: 0.0 for k in MODULES}

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info(
            'Swerve controller ready. Drive with /cmd_vel '
            '(linear.x=fwd, linear.y=strafe, angular.z=yaw).')

    def on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        for k, (x, y) in MODULES.items():
            vxi = vx - w * y
            vyi = vy + w * x
            speed = math.hypot(vxi, vyi)

            if speed < 1e-4:
                # No motion requested: hold the last heading, stop the wheel.
                angle = self.last_angle[k]
                omega = 0.0
            else:
                angle = math.atan2(vyi, vxi)
                omega = speed / WHEEL_RADIUS
                # Shortest-turn optimisation: never steer more than 90 deg;
                # instead flip the wheel and reverse the drive direction.
                if abs(wrap(angle - self.last_angle[k])) > math.pi / 2.0:
                    angle = wrap(angle + math.pi)
                    omega = -omega

            self.last_angle[k] = angle
            self.steer_pub[k].publish(Float64(data=float(angle)))
            self.drive_pub[k].publish(Float64(data=float(omega)))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
