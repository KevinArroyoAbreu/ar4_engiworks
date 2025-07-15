#!usr/bin/ python3
import rclpy
from rclpy.node import Node 
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#this is the service import
from position_tracker.srv import GetPosition

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
        self.server = self.create_service(GetPosition, '/get_position', self.handle_get_position)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.yaw = 0.0
        self.contours_green = []
        self.contours_red = []
        self.contours_blue = []

    
    
    def get_object_coordinates(self, contours, x_px_range, y_px_range, x_mm_range, y_mm_range, z0=80):
            def map_range(x, in_min, in_max, out_min, out_max):
                return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min)
            x_center = 0
            y_center = 0
            orientation = 0
            width = 0
            height = 0
            x = 0
            y = 0
            z = 100

            if len(contours) == 0:
                print("No contours found for this color.")
                return 0, 0, 100, 0, 0, 0  # Default values (z=100 means safe height)

            # Pick the largest contour
            cnt = max(contours, key=cv.contourArea)
            rect = cv.minAreaRect(cnt)
            (x_center, y_center), (width, height), orientation = rect
            #print(f'x: {x_center}, y: {y_center}')
            # Adjust orientation for tall objects
            if width < height:
                orientation = 90 + orientation

            if x_center != 0 and y_center != 0:
                y = map_range(y_center, *y_px_range, *y_mm_range)
                x = map_range(x_center, *x_px_range, *x_mm_range)
                z = z0
            else:
                x, y, z, orientation_rad, x_center, y_center = 0, 0, 100, 0, 0, 0
                print("Object not detected in expected range.")

            orientation_rad = np.deg2rad(orientation)
            return x, y, z, orientation_rad, x_center, y_center

    

    def camera_callback(self, data):
        
        try:
            cv_image_mirrored = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv_image_rotated = cv.flip(cv_image_mirrored, -1)
            cv_image_pre = cv.rotate(cv_image_rotated, cv.ROTATE_90_CLOCKWISE)
            cv_image = cv.convertScaleAbs(cv_image_pre, alpha=1, beta=0)
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
        

        #Convert to HSV
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # BLUE color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_b = np.array([110, 160, 80])
        max_hue_b = np.array([120, 255, 140])

        # RED color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_r = np.array([170, 100, 100])
        max_hue_r = np.array([180, 255, 240])

        # GREEN color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_g = np.array([74, 150, 100])
        max_hue_g = np.array([82, 255, 240])


        #========================== IMAGE PROCESSING ============================
        mask_r = cv.inRange(hsv, min_hue_r, max_hue_r)
        mask_g = cv.inRange(hsv, min_hue_g,max_hue_g)
        mask_b = cv.inRange(hsv, min_hue_b, max_hue_b)
        # mask = cv.adaptiveThreshold(mask_r, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 19, 11)
        #  cv.imshow("maskR", mask_r)
        #  cv.imshow("mask", mask)
        # Find contours
        #=========================================================================

        #============================ DRAW CONTOURS ==============================
        # contours_red, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # contours_green, _ = cv.findContours(mask_g, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # contours_blue, _ = cv.findContours(mask_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        self.contours_green, _ = cv.findContours(mask_g, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.contours_red, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.contours_blue, _ = cv.findContours(mask_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for cnt in self.contours_red:
             area = cv.contourArea(cnt)
             if area > 20:
                 cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
        for cnt in self.contours_green:
             area = cv.contourArea(cnt)
             if area > 20:
                 cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
        for cnt in self.contours_blue:
             area = cv.contourArea(cnt)
             if area > 20:
                 cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
        #=========================================================================

        
        # print(f"Number of objects detected: {len(object_detected)}")
        #print(object_detected)
        x_center = 0
        y_center = 0 
        width = 0
        height = 0 
        orientation = 0
        # ADJUST THIS: this is fixed (due to 2d camera being used)
        z0 = 80.0 #using 20mm cubes



        def draw_orientation_box(contours, color, image):
            for cnt in contours:
                area = cv.contourArea(cnt)
                if area < 20:
                    continue

                rect = cv.minAreaRect(cnt)
                (x_center, y_center), (width, height), orientation = rect

                # Draw rotated bounding box
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cv.drawContours(image, [box], 0, color, 2)

                # Draw arrow for orientation
                center = (int(x_center), int(y_center))
                length = 40  # Length of arrow
                radian = np.deg2rad(orientation)
                end_point = (int(center[0] + length * np.cos(radian)),
                            int(center[1] + length * np.sin(radian)))
                cv.arrowedLine(image, center, end_point, color, 2)

        draw_orientation_box(self.contours_green, (0, 255, 0), cv_image)
        draw_orientation_box(self.contours_red, (0, 0, 255), cv_image)
        draw_orientation_box(self.contours_blue, (255, 0, 0), cv_image)


        #======================== COORDINATE CALCULATION ==========================

        
        x_px_range = (46, 427)
        y_px_range = (146, 420)
        x_mm_range = (-197, 187)
        y_mm_range = (-295, -588)

        x, y, z, yaw, x_px, y_px = self.get_object_coordinates(self.contours_green, x_px_range, y_px_range, x_mm_range, y_mm_range)


        self.x_pos = x
        self.y_pos = y
        self.z_pos = z
        self.yaw = yaw

        cv.putText(cv_image, f"x: {round(x,1)} y: {round(y,1)}", (int(x_px), int(y_px)),
           cv.FONT_HERSHEY_PLAIN, 1, (9, 9, 9), 1)
        cv.circle(cv_image, (int(x_px), int(y_px)), 3, (0, 255, 0), -1)


        cv.imshow("Obj Detection - Real World Coordinates", cv_image)
        cv.waitKey(1)

    # Handle for the service to get position
    def handle_get_position(self, request, response):
        color = request.color_requested.lower()

        if color == "green":
            contours = self.contours_green
        elif color == "red":
            contours = self.contours_red
        elif color == "blue":
            contours = self.contours_blue
        else:
            contours = []

        x_px_range = (46, 427)
        y_px_range = (146, 420)
        x_mm_range = (-197, 187)
        y_mm_range = (-295, -588)

        x, y, z, yaw, _, _ = self.get_object_coordinates(contours, x_px_range, y_px_range, x_mm_range, y_mm_range)

        response.x_position = float(x)
        response.y_position = float(y)
        response.z_position = float(z)
        response.yaw = float(yaw)

        self.get_logger().info(
            f"Position requested: color: {color} x={x}, y={y}, z={z}, yaw={yaw}"
        )

        return response



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
