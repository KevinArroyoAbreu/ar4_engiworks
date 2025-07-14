import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node 
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WriteImage(Node):
    def __init__(self):
        super().__init__('write_image_node')

        self.subscription = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.camera_callback,
            10
        )
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image_mirrored = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv_image = cv2.flip(cv_image_mirrored, -1)
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
        
        folder_path = 'src/opencv_pkg/images/output/'
        img = cv2.imwrite(folder_path + 'object_new_position.jpg', cv_image)
        cv2.imshow("Captured Image", cv_image)
        cv2.waitKey(0)


def main(args=None):
    rclpy.init(args=args)
    write_image_object = WriteImage()
    
    try:
        rclpy.spin(write_image_object)
    except KeyboardInterrupt:
        print("Shutting down write_image_node")
    finally:
        write_image_object.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()