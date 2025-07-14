import cv2
import numpy as np

hsv_image = None  # global variable to store the current HSV frame

def mouse_callback(event, x, y, flags, param):
    global hsv_image
    if event == cv2.EVENT_LBUTTONDOWN:
        if hsv_image is not None:
            hsv_val = hsv_image[y, x]
            print(f'Clicked pixel at ({x}, {y}) HSV: {hsv_val}')

def main():
    global hsv_image
    cap = cv2.VideoCapture(4)  # Change 0 if you have multiple cameras
    if not cap.isOpened():
        print("Cannot open camera")
        return

    cv2.namedWindow('Live Camera')

    cv2.setMouseCallback('Live Camera', mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame. Exiting...")
            break

        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cv2.imshow('Live Camera', frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
