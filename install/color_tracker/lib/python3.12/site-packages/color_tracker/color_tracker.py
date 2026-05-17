"""
color_detector.py
-----------------
ROS2 Node: ColorDetectorNode

Responsibilities:
  - Subscribe to raw camera images from Turtlebot3
  - Use OpenCV to detect the neon-green sphere
  - Compute horizontal error (sphere center vs image center)
  - Publish detection results as Float32MultiArray:
      data[0] = error in pixels  (0.0 if not detected)
      data[1] = detected flag    (1.0 = detected, 0.0 = not detected)
      data[2] = contour area     (0.0 if not detected)
      data[3] = last known dir   (+1.0 = was on RIGHT, -1.0 = LEFT, 0.0 = never seen)

Run:
    python3 color_detector.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


# ──────────────────────────────────────────────────────────
#  HSV range for NEON green sphere
#  High saturation + high brightness so dull map greens
#  (walls, floor markings) are automatically rejected
# ──────────────────────────────────────────────────────────
LOWER_GREEN = (50, 180, 180)
UPPER_GREEN = (75, 255, 255)

# Ignore contours smaller than this (noise filter)
MIN_CONTOUR_AREA = 500

# Morphological kernel to clean up mask
MORPH_KERNEL = np.ones((5, 5), np.uint8)


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        # Updated from the first received frame
        self.image_width  = 640
        self.image_height = 480

        # Remembers which side the sphere was last seen on
        # Used by navigator to decide recovery rotation direction
        # +1.0 = sphere was on RIGHT, -1.0 = LEFT, 0.0 = never seen
        self.last_known_direction = 0.0

        # ── Publisher: sends detection info to navigator ──
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/color_detector/detection',
            10
        )

        # ── Subscriber: raw camera frames ────────────────
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("ColorDetectorNode started — waiting for frames...")

    # ──────────────────────────────────────────────────────
    #  IMAGE CALLBACK — runs every time a frame arrives
    # ──────────────────────────────────────────────────────
    def image_callback(self, msg):

        # Step 1: Convert ROS Image → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        # Update dimensions from actual frame size
        self.image_height, self.image_width = frame.shape[:2]
        image_center_x = self.image_width // 2

        # Step 2: BGR → HSV (better for colour thresholding)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Step 3: Create binary mask — white pixels = neon green
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        # Step 4: Morphological cleanup
        #   erode  — removes small noise blobs
        #   dilate — fills gaps in sphere mask
        mask = cv2.erode(mask,  MORPH_KERNEL, iterations=1)
        mask = cv2.dilate(mask, MORPH_KERNEL, iterations=2)

        # Step 5: Find contours on cleaned mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detection_msg = Float32MultiArray()

        if contours:
            # Pick the largest contour — most likely the sphere
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area > MIN_CONTOUR_AREA:
                M = cv2.moments(largest)

                if M["m00"] != 0:
                    # ── SPHERE DETECTED ──────────────────
                    cx = int(M["m10"] / M["m00"])   # sphere center x
                    cy = int(M["m01"] / M["m00"])   # sphere center y

                    # Error: positive = sphere is LEFT of center (need to rotate left)
                    #        negative = sphere is RIGHT of center (need to rotate right)
                    # (Same convention as your working code: image_center_x - cx)
                    error = float(image_center_x - cx)

                    # Update last known direction for recovery
                    # If error < 0, sphere is to the RIGHT
                    self.last_known_direction = -1.0 if error < 0 else 1.0

                    detection_msg.data = [
                        error,
                        1.0,          # detected = True
                        float(area),
                        self.last_known_direction
                    ]

                    self.get_logger().info(
                        f"Detected | cx={cx} | error={error:.1f}px | area={area:.0f}"
                    )

                    # ── Debug visuals ────────────────────
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                    cv2.line(frame,
                             (image_center_x, 0),
                             (image_center_x, self.image_height),
                             (255, 0, 0), 2)
                    cv2.putText(frame,
                                f"Error: {error:.0f}",
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, (255, 255, 255), 2)

                else:
                    # Moments failed — treat as not detected
                    detection_msg.data = self._make_lost_msg()
            else:
                # Contour too small — noise
                detection_msg.data = self._make_lost_msg()
        else:
            # ── SPHERE OUT OF FRAME ──────────────────────
            detection_msg.data = self._make_lost_msg()
            cv2.putText(frame,
                        "Searching for sphere...",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2)
            self.get_logger().info("Sphere not in frame — passing last direction to navigator")

        self.detection_pub.publish(detection_msg)

        # Show camera and mask windows
        cv2.imshow("Color Tracking", frame)
        cv2.imshow("Green Mask", mask)
        cv2.waitKey(1)

    # ──────────────────────────────────────────────────────
    #  Build a "not detected" message.
    #  Passes last_known_direction in data[3] so the
    #  navigator knows which way to rotate to recover.
    # ──────────────────────────────────────────────────────
    def _make_lost_msg(self):
        return [0.0, 0.0, 0.0, self.last_known_direction]


# ──────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
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