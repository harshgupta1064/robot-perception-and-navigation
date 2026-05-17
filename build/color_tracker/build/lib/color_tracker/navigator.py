import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Control parameters
        self.Kp = 0.005
        self.max_angular = 2.0
        self.linear_speed = 0.15
        self.error_threshold = 25
        self.recovery_speed = 0.35
        self.timeout_sec = 1.5

        self.stop_area = 400000

        self.error = 0.0
        self.detected = False
        self.area = 0.0
        self.last_dir = 0.0
        self.stopped = False   # latching stop flag

        self.last_msg_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/detection',
            self.detection_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Navigator Node Started")
        self.get_logger().info(f"  Stop area threshold: {self.stop_area}")
        self.get_logger().info(f"  Error threshold (px): {self.error_threshold}")

    def detection_callback(self, msg):
        if len(msg.data) < 4:
            return

        self.last_msg_time = self.get_clock().now()

        self.error    = msg.data[0]
        self.detected = bool(msg.data[1])
        self.area     = msg.data[2]
        self.last_dir = msg.data[3]

        # If sphere is lost after stopping, allow movement again
        if not self.detected:
            self.stopped = False

    def control_loop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        elapsed = (
            self.get_clock().now() - self.last_msg_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout_sec:
            twist.twist.linear.x  = 0.0
            twist.twist.angular.z = self.recovery_speed
            self.cmd_pub.publish(twist)
            self.get_logger().warn("Timeout — no detection, spinning to search")
            return

        if self.detected:
            self.track_target(twist)
        else:
            self.recover_target(twist)

        self.cmd_pub.publish(twist)

    def track_target(self, twist):

        if self.area >= self.stop_area and abs(self.error) < self.error_threshold:
            self.stopped = True

        if self.stopped:
            twist.twist.linear.x  = 0.0
            twist.twist.angular.z = 0.0
            self.get_logger().info(
                f"STOPPED | Area={self.area:.0f} | Error={self.error:.0f}"
            )
            return

        angular = self.Kp * self.error
        angular = max(min(angular, self.max_angular), -self.max_angular)
        twist.twist.angular.z = angular

        if abs(self.error) < self.error_threshold:
            twist.twist.linear.x = self.linear_speed
            self.get_logger().info(
                f"FORWARD | Error={self.error:.0f}px | Area={self.area:.0f}"
            )
        else:
            twist.twist.linear.x = 0.0
            self.get_logger().info(
                f"ROTATING | Error={self.error:.0f}px | angular.z={angular:.3f}"
            )

    def recover_target(self, twist):
        twist.twist.linear.x = 0.0

        if self.last_dir > 0:
            twist.twist.angular.z = self.recovery_speed
            self.get_logger().info("RECOVERY: rotating LEFT")
        elif self.last_dir < 0:
            twist.twist.angular.z = -self.recovery_speed
            self.get_logger().info("RECOVERY: rotating RIGHT")
        else:
            twist.twist.angular.z = self.recovery_speed
            self.get_logger().info("RECOVERY: slow spin searching")

    def stop_robot(self):
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp    = self.get_clock().now().to_msg()
        stop_cmd.twist.linear.x  = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info("Robot stopped safely.")


def main(args=None):
    rclpy.init(args=args)

    node = Navigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()