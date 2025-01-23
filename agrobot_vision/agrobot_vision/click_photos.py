import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/gemini_e/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.image_counter = 1
        self.current_frame = None

        # Create dataset folder if it doesn't exist
        if not os.path.exists('dataset'):
            os.makedirs('dataset')

        # Create a window and set the mouse callback function
        cv2.namedWindow('Camera')
        cv2.setMouseCallback('Camera', self.click_event)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Display the frame
        cv2.imshow('Camera', self.current_frame)
        cv2.waitKey(1)

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            # Save the image
            image_path = f'dataset/image_{self.image_counter}.jpg'
            cv2.imwrite(image_path, self.current_frame)
            print(f'Image saved: {image_path}')
            self.image_counter += 1


def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()

    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass

    image_saver.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
