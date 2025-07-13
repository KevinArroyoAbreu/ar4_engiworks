import rclpy
import cv2

class LoadImage:
    def __init__(self, picture):
        self.picture = picture

    def read_photo(self):
        print(f"Loading image from: {self.picture}")
        img = cv2.imread(self.picture)
        if img is None:
            print(f"Error: Could not load image from {self.picture}")
            return
        print(f"Image shape: {img.shape}")
        cv2.imshow("image", img)
        print("Image window should now be open.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("Image window closed.")


def main():
    image = '/home/karroyabreu/ar4/src/opencv/images/input/Robot-Arm-PNG-HD-Quality.png'
    load_image = LoadImage(image)
    load_image.read_photo()

if __name__ == '__main__':
    main()