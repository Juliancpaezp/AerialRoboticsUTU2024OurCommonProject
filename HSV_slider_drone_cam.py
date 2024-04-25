import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

class HSVAdjustmentNode(Node):
    def __init__(self):
        super().__init__('hsv_adjustment_node')
        self.bridge = CvBridge()

        # Create subscriber for receiving image data from the drone camera
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)

        # Create client for sending takeoff and land commands to the drone
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')

        # Create publisher for sending Twist messages
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available yet, waiting...')  

        # Create request message for takeoff command
        self.req = TelloAction.Request()
        self.req.cmd = 'takeoff'  # Set the command to 'takeoff'
        self.future = self.cli.call_async(self.req)
        self.get_logger().info("Attempting to take off...")

        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        # Display the live image with sliders for adjusting HSV values
        self.adjust_hsv(hsv)

    def nothing(self, x):
        pass

    def adjust_hsv(self, image):
        cv2.namedWindow('Adjust HSV')

        # Create trackbars for adjusting HSV values
        cv2.createTrackbar('Hue Min', 'Adjust HSV', 0, 179, self.nothing)
        cv2.createTrackbar('Hue Max', 'Adjust HSV', 179, 179, self.nothing)
        cv2.createTrackbar('Saturation Min', 'Adjust HSV', 0, 255, self.nothing)
        cv2.createTrackbar('Saturation Max', 'Adjust HSV', 255, 255, self.nothing)
        cv2.createTrackbar('Value Min', 'Adjust HSV', 0, 255, self.nothing)
        cv2.createTrackbar('Value Max', 'Adjust HSV', 255, 255, self.nothing)

        while True:
            # Get current trackbar positions
            h_min = cv2.getTrackbarPos('Hue Min', 'Adjust HSV')
            h_max = cv2.getTrackbarPos('Hue Max', 'Adjust HSV')
            s_min = cv2.getTrackbarPos('Saturation Min', 'Adjust HSV')
            s_max = cv2.getTrackbarPos('Saturation Max', 'Adjust HSV')
            v_min = cv2.getTrackbarPos('Value Min', 'Adjust HSV')
            v_max = cv2.getTrackbarPos('Value Max', 'Adjust HSV')

            # Create HSV lower and upper bounds arrays
            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])

            # Convert image to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Create a mask using the HSV lower and upper bounds
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            # Apply the mask to the original image
            result = cv2.bitwise_and(image, image, mask=mask)

            # Display the resulting image
            cv2.imshow('Result', result)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    hsv_adjustment_node = HSVAdjustmentNode()
    rclpy.spin(hsv_adjustment_node)
    hsv_adjustment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()