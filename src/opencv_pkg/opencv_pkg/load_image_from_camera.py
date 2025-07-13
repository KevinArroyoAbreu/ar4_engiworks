import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class LoadImage(Node):
    def __init__(self):
        super().__init__('load_image_node')

        self.subscription = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.camera_callback,
            10
        )
        self.bridge = CvBridge()

        # Output path setup
        self.output_dir = '/home/karroyabreu/ar4/src/opencv_pkg/images/output'
        os.makedirs(self.output_dir, exist_ok=True)

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.get_logger().info("Image received.")

            # Save image
            image_path = os.path.join(self.output_dir, 'camera_image.png')
            cv2.imwrite(image_path, cv_image)

            # Show image
            cv2.imshow("frame from camera", cv_image)
            cv2.waitKey(1)  # Use 1 instead of 0 to avoid blocking

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LoadImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
