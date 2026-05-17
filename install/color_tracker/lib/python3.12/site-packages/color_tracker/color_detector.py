import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')

        self.bridge = CvBridge()

        self.image_width = 640
        self.image_height = 480

        self.last_known_direction = 0.0

        # Wider green range for better robustness
        self.lower_green = np.array([35, 80, 40])
        self.upper_green = np.array([90, 255, 255])

        self.min_area = 400
        self.min_circularity = 0.8

        self.kernel = np.ones((5, 5), np.uint8)

        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/detection',
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("ColorTracker Node Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        self.image_height, self.image_width = frame.shape[:2]
        image_center_x = self.image_width // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(
            hsv,
            self.lower_green,
            self.upper_green
        )

        # Morphological cleanup
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        best_contour = None
        best_circularity = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_area:
                continue

            perimeter = cv2.arcLength(contour, True)

            if perimeter == 0:
                continue

            circularity = (
                4 * math.pi * area
            ) / (perimeter ** 2)

            if (
                circularity > self.min_circularity
                and circularity > best_circularity
            ):
                best_circularity = circularity
                best_contour = contour

        detection_msg = Float32MultiArray()

        if best_contour is not None:
            M = cv2.moments(best_contour)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                area = cv2.contourArea(best_contour)

                error = float(image_center_x - cx)

                self.last_known_direction = (
                    1.0 if error > 0 else -1.0
                )

                detection_msg.data = [
                    error,
                    1.0,
                    area,
                    self.last_known_direction
                ]

                # Visualization
                cv2.circle(
                    frame,
                    (cx, cy),
                    8,
                    (0, 0, 255),
                    -1
                )

                cv2.drawContours(
                    frame,
                    [best_contour],
                    -1,
                    (0, 255, 0),
                    2
                )

                (x, y), radius = cv2.minEnclosingCircle(
                    best_contour
                )

                cv2.circle(
                    frame,
                    (int(x), int(y)),
                    int(radius),
                    (255, 255, 0),
                    2
                )

                cv2.line(
                    frame,
                    (image_center_x, 0),
                    (image_center_x, self.image_height),
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Error: {error:.0f} | Area: {area:.0f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2
                )

        else:
            detection_msg.data = [
                0.0,
                0.0,
                0.0,
                self.last_known_direction
            ]

            cv2.putText(
                frame,
                "Searching...",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        self.detection_pub.publish(detection_msg)

        cv2.imshow("Color Tracking", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ColorTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()