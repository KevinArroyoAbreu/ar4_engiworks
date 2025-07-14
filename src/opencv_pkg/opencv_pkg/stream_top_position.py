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
            cv_image_rotated = cv.flip(cv_image_mirrored, -1)
            cv_image = cv.rotate(cv_image_rotated, cv.ROTATE_90_CLOCKWISE)
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
        

        #Convert to HSV
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # BLUE color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue = np.array([110, 160, 80])
        max_hue = np.array([120, 255, 140])

        mask_r = cv.inRange(hsv, min_hue, max_hue)
        mask = cv.adaptiveThreshold(mask_r, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 19, 11)
        cv.imshow("mask", mask_r)

        # Find contours

        contours, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
       # print("contours: ", contours)

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

       # print(f"Number of objects detected: {len(object_detected)}")
        #print(object_detected)


        #Pixel to mm conversion 
        x_pxl_center = 238
        y_pxl_center = 307.5# not needed (y0 is not on the feed)
        pxl_mm_conversion = 18.5 / 20  # 18 pixels corresponds to 20 mm

        # Using positions read using rviz and pose_gui node
        x0 = 0
        y0 = 47.6
        calib_y = 20#an offset to correct y position
        # ADJUST THIS: this is fixed (due to 2d camera being used)
        z0 = 10 #using 20mm cubes


        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center,y_center), (width, height), orientation = rect
            #print('width: ', width, ' height: ', height)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(cv_image, [box], 0, (255, 0, 0), 1)


         #condition right of the imgage
        if (x_center > x_pxl_center):
            y =   y0 + (y_center)/pxl_mm_conversion + calib_y
            x = x0+(-x_pxl_center + x_center)/pxl_mm_conversion
        
        #condition left of the imgage
        elif (x_center < x_pxl_center):
            y =  y0 + (y_center)/pxl_mm_conversion + calib_y
            x = x0-(x_pxl_center - x_center)/pxl_mm_conversion 

        elif (x_center == x_pxl_center) and (y_center == y_pxl_center):
            x= x0
            y= y0
        
        else:
            y = 20
            x = 0
            print("Object not detected in the expected range, setting default coordinates.")


        cv.putText(cv_image, "x: {}".format(round(x, 1)) + " y: {}".format(round(y,1)), 
                       (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0,0,255),1)
        cv.circle(cv_image, (int(x_center), int(y_center)), 1, (255, 0, 0), -1)
        cv.imshow("Obj Detection - Real World Coordinates", cv_image)
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
