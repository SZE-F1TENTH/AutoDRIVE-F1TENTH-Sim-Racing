#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Bool


class DriveConverter(Node):
    def __init__(self):
        super().__init__('ackermann_to_teleop_converter')

        qos = QoSProfile(depth=10)

        # Subscriber az /drive topicra
        self.sub_drive = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            qos
        )

        # Publisher-ek a teleop által használt topicokra
        self.pub_throttle = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', qos)
        self.pub_steering = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', qos)
        self.pub_reset = self.create_publisher(Bool, '/autodrive/reset_command', qos)

        self.get_logger().info("Ackermann → Teleop konverter elindult.")

    def drive_callback(self, msg: AckermannDriveStamped):
        throttle_msg = Float32()
        steering_msg = Float32()
        reset_msg = Bool()

        # AckermannDriveStamped: msg.drive.speed és msg.drive.steering_angle a lényeg
        throttle_msg.data = float(msg.drive.speed) * 0.02 # előre-hátra sebesség
        steering_msg.data = float(msg.drive.steering_angle)  # kormány szög
        reset_msg.data = False  # alapértelmezetten nincs reset

        # Publikálás
        self.pub_throttle.publish(throttle_msg)
        self.pub_steering.publish(steering_msg)
        self.pub_reset.publish(reset_msg)

        self.get_logger().debug(f"Drive átalakítva -> Throttle: {throttle_msg.data:.2f}, Steering: {steering_msg.data:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = DriveConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
