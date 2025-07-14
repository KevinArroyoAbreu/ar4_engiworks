import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorFilter(Node):

    def __init__(self):
        super().__init__('color_filter_node')

        self.subscription = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.camera_callback,
            10
        )
        self.bridge = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return  # Exit if conversion fails

        # Convert to HSV (OpenCV uses BGR by default)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # RED color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue = np.array([0, 120, 120])
        max_hue = np.array([210, 240, 240])

        # Create mask
        mask_r = cv2.inRange(hsv, min_hue, max_hue)

        # Apply mask to original image
        res_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_r)

        # Display images
        cv2.imshow("Original Image", cv_image)
        cv2.imshow("Color Mask", mask_r)
        cv2.imshow("Filtered Image", res_r)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)  # Initialize first
    color_filter_object = ColorFilter()
    try:
        rclpy.spin(color_filter_object)
    except KeyboardInterrupt:
        print("Keyboard Interrupt, shutting down...")
    finally:
        color_filter_object.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
