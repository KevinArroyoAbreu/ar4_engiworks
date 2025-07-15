#!usr/bin/ python3
import rclpy
from rclpy.node import Node 
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('edge_detection_node')

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
            cv_image = cv.flip(cv_image_mirrored, -1)
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
        
        #crop for object detection
        #cropped_img = cv_image[160:410, 180:500]  # [x:x, y:y]
        

        # Convert to grayscale
        #gray = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)
        #cv.imshow("Grayscale Image", gray)

        # Create mask
        #(block_size, C) (last 2 params)
        #higher blockSize = wider local neighbor, smoother lighting
        #lower C = more aggressive thresholding
        #mask = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 11, 25)
        #mask = cv.bitwise_not(mask)
        #cv.imshow("Adaptive Threshold Mask", mask)

        #Convert to HSV
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # BKUE color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue = np.array([110, 160, 80])
        max_hue = np.array([120, 255, 140])

        mask_r = cv.inRange(hsv, min_hue, max_hue)
        mask = cv.adaptiveThreshold(mask_r, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 19, 11)
        cv.imshow("mask", mask_r)

        # Find contours

        contours, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cv_image, [cnt], True, [0, 255, 255], 3)

        #if contours:
         #  largest_contour = max(contours, key=cv.contourArea)
          # cv.drawContours(cv_image, [largest_contour], -1, (255, 255, 0), 3)


        object_detected = []

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 20:
                cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
                object_detected.append(cnt)

        print(f"Number of objects detected: {len(object_detected)}")
        #print(object_detected)

        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center,y_center), (width, height), orientation = rect
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(cv_image, [box], 0, (255, 0, 0), 1)
            cv.putText(cv_image, f"X: {x_center:.2f}, Y: {y_center:.2f}", (int(x_center), int(y_center)), cv.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
            cv.circle(cv_image, (int(x_center), int(y_center)), 2, (0, 0, 255), -1)

        cv.imshow("Feed", cv_image)
        #cv.imshow("Original Image", cv_image)
        cv.waitKey(1)


def main(args=None):   
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    try:
        rclpy.spin(object_detection)
    except KeyboardInterrupt:
        print("Keyboard Interrupt, shutting down...")
    finally:
        object_detection.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()
