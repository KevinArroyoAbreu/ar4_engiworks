#!usr/bin/ python3


# A copy of the original position_server.py, 
# but with targets in container added

import rclpy
from rclpy.node import Node 
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#this is the service import
from position_tracker.srv import GetPosition

req = GetPosition.Request()
print(hasattr(req, "desired_z")) 

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('edge_detection_node_v2')

        self.subscription = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.camera_callback,
            10
        )
        self.last_pose_by_color = {}
        self.bridge_object = CvBridge()
        self.server = self.create_service(GetPosition, '/get_position_v2', self.handle_get_position)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.yaw = 0.0
        self.objects_green = []
        self.objects_red = []
        self.objects_blue = []
        # List for the targets
        self.targets_green = []
        self.targets_red = []
        self.targets_blue = []

    
    
    def get_object_coordinates(self, contours, x_px_range, y_px_range, x_mm_range, y_mm_range, z0):
            def map_range(x, in_min, in_max, out_min, out_max):
                return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min)
            x_center = 0
            y_center = 0
            orientation = 0
            width = 0
            height = 0
            x = 0
            y = 0
            z = 250

            if len(contours) == 0:
                print("No contours found for this color.")
                return 0, 0, 250, 0, 0, 0  # Default values (z=250 means safe height)

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
                x, y, z, orientation_rad, x_center, y_center = 0, 0, 250, 0, 0, 0
                print("Object not detected in expected range.")

            orientation_rad = np.deg2rad(orientation)
            return x, y, z, orientation_rad, x_center, y_center

    

    def camera_callback(self, data):
        
        try:
            cv_image_mirrored = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv_image_rotated = cv.flip(cv_image_mirrored, -1)
            cv_image_pre = cv.rotate(cv_image_rotated, cv.ROTATE_90_CLOCKWISE)
            cv_imageUnPr = cv.convertScaleAbs(cv_image_pre, alpha=1.4, beta=00)
            kernel = np.ones((3, 3), np.uint8)
            cv_image = cv.morphologyEx(cv_imageUnPr, cv.MORPH_OPEN, kernel)
            cv_image = cv.morphologyEx(cv_imageUnPr, cv.MORPH_CLOSE, kernel)
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
        

        #Convert to HSV
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # BLUE color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_b = np.array([110, 100, 100])
        max_hue_b = np.array([130, 255, 255])

        # RED color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_r = np.array([150, 100, 100])
        max_hue_r = np.array([225, 255, 255])

        # GREEN color range (adjust these if needed) (Hue, Saturation, Value)
        min_hue_g = np.array([70, 100, 100])
        max_hue_g = np.array([80, 255, 255])


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
        # objects_red, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # objects_green, _ = cv.findContours(mask_g, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # objects_blue, _ = cv.findContours(mask_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        self.objects_green, _ = cv.findContours(mask_g, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.objects_red, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.objects_blue, _ = cv.findContours(mask_b, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


        #Separate the targets from the objects using aspect ratio analysis
        def split_targets(contours):
            targets = []
            cubes = []
            for cnt in contours:
                if cv.contourArea(cnt) < 10:
                    continue
                # Updated minAreaRect to ensure Correct Orientation
                #      and prevent flickering
                rect = cv.minAreaRect(cnt)
                (x, y), (w, h), angle = rect

                # Ensure w is always the longer side
                if w < h:
                    w, h = h, w
                    angle += 90

                aspect_ratio = max(w, h) / min(w, h) if min(w, h) != 0 else 1.0
                if aspect_ratio > 1.5:  # Adjust threshold as needed
                    targets.append(cnt)
                else:
                    cubes.append(cnt)
            return cubes, targets

        self.objects_red, self.targets_red = split_targets(self.objects_red)
        self.objects_green, self.targets_green = split_targets(self.objects_green)
        self.objects_blue, self.targets_blue = split_targets(self.objects_blue)



        for cnt in self.objects_red:
             area = cv.contourArea(cnt)
             if area > 20:
                 cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
        for cnt in self.objects_green:
             area = cv.contourArea(cnt)
             if area > 20:
                 cnt = cv.approxPolyDP(cnt, 0.03 * cv.arcLength(cnt, True), True)
        for cnt in self.objects_blue:
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
        # = 80.0 #using 20mm cubes
        # RANGE BEING DETECTED ============
        # RANGES DEFINED HERE AND ON ANOTHER PLACE ALSO
        x_px_range = (46, 427)
        y_px_range = (146, 420)

        y_px_range_targets = (420, 530)
        
        x_mm_range = (-197, 187)
        y_mm_range = (-295, -588)

        y_mm_range_targets = (-588, -705.6)

        def draw_orientation_box(contours, color, image, x_px_range, y_px_range):
            for cnt in contours:
                area = cv.contourArea(cnt)
                if area < 20:
                    continue

                rect = cv.minAreaRect(cnt)
                (x_center, y_center), (width, height), orientation = rect

                # BOUNDING BOX FOR DRAWING THE BOXES ====================================================
                # =======================================================================================

                # Skip drawing if outside bounding box
                if not (x_px_range[0] <= x_center <= x_px_range[1] and y_px_range[0] <= y_center <= y_px_range[1]):
                    continue

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

        draw_orientation_box(self.objects_green, (0, 255, 0), cv_image, x_px_range, y_px_range)
        draw_orientation_box(self.objects_red, (0, 0, 255), cv_image, x_px_range, y_px_range)
        draw_orientation_box(self.objects_blue, (255, 0, 0), cv_image, x_px_range, y_px_range)

        # Draw targets (ADJUST THE RANGE)
        draw_orientation_box(self.targets_green, (0, 200, 0), cv_image, x_px_range, y_px_range_targets)
        draw_orientation_box(self.targets_red, (0, 0, 200), cv_image, x_px_range, y_px_range_targets)
        draw_orientation_box(self.targets_blue, (200, 0, 0), cv_image, x_px_range, y_px_range_targets)

        #======================== COORDINATE CALCULATION ==========================

        

        # Define your contour-color pairs
        colors = [
            (self.objects_green, (0, 30, 0), "Green"),
            (self.objects_red, (0, 0, 30), "Red"),
            (self.objects_blue, (30, 0, 0), "Blue"),
        ]

        # Loop over each color group
        for contours, draw_color, label in colors:
            x, y, z, yaw, x_px, y_px = self.get_object_coordinates(
                contours, x_px_range, y_px_range, x_mm_range, y_mm_range, 250.0
            )
            # BOUNDING BOX IS HERE TOO FOR DRAWING =======================================================
            # ============================================================================================
            if x is not None and y is not None:
                if x_px_range[0] <= x_px <= x_px_range[1] and y_px_range[0] <= y_px <= y_px_range[1]:
                    # Draw a circle and text at the object location only if inside bounding box
                    cv.circle(cv_image, (int(x_px), int(y_px)), 3, draw_color, -1)
                    cv.putText(cv_image,
                            f"{label}: x={round(x, 1)} y={round(y, 1)}",
                            (int(x_px) + 5, int(y_px) - 5),
                            cv.FONT_HERSHEY_PLAIN, 1, draw_color, 1)
                else:
                    self.get_logger().warn(f"{label} object outside bounding box; skipping draw.")

        #Draw boxes for targets
        target_sets = [
            (self.targets_green, (0, 200, 0), "Green Target"),
            (self.targets_red, (0, 0, 200), "Red Target"),
            (self.targets_blue, (200, 0, 0), "Blue Target"),
        ]

        for contours, draw_color, label in target_sets:
            x, y, z, yaw, x_px, y_px = self.get_object_coordinates(
                contours, x_px_range, y_px_range, x_mm_range, y_mm_range, 250.0
            )
            if x is not None and y is not None:
                if x_px_range[0] <= x_px <= x_px_range[1] and y_px_range_targets[0] <= y_px <= y_px_range_targets[1]:
                    cv.circle(cv_image, (int(x_px), int(y_px)), 3, draw_color, -1)
                    cv.putText(cv_image,
                            f"{label}: x={round(x, 1)} y={round(y, 1)}",
                            (int(x_px) + 5, int(y_px) - 5),
                            cv.FONT_HERSHEY_PLAIN, 1, draw_color, 1)


        cv.imshow("Obj Detection - Real World Coordinates", cv_image)
        cv.waitKey(1)

    # Handle for the service to get position
    def handle_get_position(self, request, response):
        color = request.color.lower()
        z0 = request.desired_z if request.desired_z > 0 else 250.0

        if color == "green":
            contours = self.objects_green
        elif color == "red":
            contours = self.objects_red
        elif color == "blue":
            contours = self.objects_blue
        elif color == "green_target":
            contours = self.targets_green
        elif color == "red_target":
            contours = self.targets_red
        elif color == "blue_target":
            contours = self.targets_blue
        else:
            contours = []
        # RANGES DEFINED HERE AND ON ANOTHER PLACE ALSO
        x_px_range = (46, 427)
        y_px_range = (146, 420)

        y_px_range_targets = (420, 530)
        
        x_mm_range = (-197, 187)
        y_mm_range = (-295, -588)

        y_mm_range_targets = (-588, -705.6)

        if color in self.last_pose_by_color:
            x, y, yaw, x_px, y_px = self.last_pose_by_color[color]
            z = request.desired_z if request.desired_z > 0 else 250.0
        else:
            if "target" in color:
                x, y, z, yaw, x_px, y_px = self.get_object_coordinates(
                    contours, x_px_range, y_px_range_targets, x_mm_range, y_mm_range_targets, z0)
            else:
                x, y, z, yaw, x_px, y_px = self.get_object_coordinates(
                    contours, x_px_range, y_px_range, x_mm_range, y_mm_range, z0)

        # BOUNDING BOX FOR DETECTION ==========================================================================
        # =====================================================================================================

        if not (x_px_range[0] <= x_px <= x_px_range[1] and y_px_range[0] <= y_px <= y_px_range[1]):
            self.get_logger().warn("Detected object is outside of the bounding box; ignoring.")
            return response  # Optionally return empty/default response or raise an error

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
