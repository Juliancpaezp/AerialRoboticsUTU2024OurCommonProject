"""
# Aerial Robotics UTU 2024: Our Common Project
 
## Description:
This code is meant to take off a trello drone, then move trough 
green check points using images from a on-board camera in a 
gazebo simulation, using ROS2 and OpenCV.

## Installation:
Based on: https://github.com/TIERS/drone_racing_ros2/tree/main
Run this on your console once: (or VisualStudio Code terminal, if you rather)

    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/TIERS/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build

## Usage
Run this every time you want to run the gazeboenvironment

cd ~/drone_racing_ros2_ws
source install/setup.bash
export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
source /usr/share/gazebo/setup.sh
ros2 launch tello_gazebo simple_launch.py

Then, run "OpenProjectAerialRobotics.py" with python from console

## Extra
You can also add the path to to your ".bashrc" file, if you want to run the python file faster...

    nano ~/.bashrc

And then add:

    # Drone_Racing_ROS2
    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    export GAZEBO_MODEL_PATH=/home/julian/drone_racing_ros2_ws/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

## Control the Drone

    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1    

Enjoy!
 
## Authors: 
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Michalis Iona [michalis.l.iona@utu.fi]
     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
 
For: University of Turku - TIERS 
Course: Aerial Robotics and Multi-Robot Systems 
Date: March 26th, 2024 

## Pending
- La condicion de avanzar puede ser solo si el centro del area mas grande no esta cerca de los magenes, en lugar de ignorar el contorno, asi se mantiene activada la colision avoidances

"""

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tello_msgs.srv import TelloAction
import time
from geometry_msgs.msg import Twist
import numpy as np

class OpenProjectAerialRobotics:
    def __init__(self):
        print("Initializing node...")
        self.node = rclpy.create_node('OpenProjectAerialRobotics')
        self.bridge = CvBridge()

        # Create subscriber for receiving image data from the drone camera
        print("Subscribing to camera service...")
        self.subscription = self.node.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        
        # Create client for sending takeoff and land commands to the drone
        self.cli = self.node.create_client(TelloAction, '/drone1/tello_action')

        # Create publisher for sending Twist messages
        self.publisher_ = self.node.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available yet, waiting...')  
        
        # Create request message for takeoff command
        self.req = TelloAction.Request()
        self.req.cmd = 'takeoff'  # Set the command to 'takeoff'
        self.future = self.cli.call_async(self.req)
        print("Attempting to take off...")
        time.sleep(4)  # give a little time to little parrot drone to take off



    def image_callback(self, msg):

        ## User-modifiable variables
        new_width = 450                 # Height of incoming camera image
        new_height = 300                # Height of incoming camera image
        allowed_centering_error = 17    # Maximum error while centering drone on gate
        margin = 20                     # Pixels to ignore in borders of image
        land_at_STOP = 8600             # Area of STOP contour to land drone 




        ## Doing image analisys ##

        # Convert ROS image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Resize the image
        image = cv2.resize(image, (new_width, new_height))

        # Convert color from BGR to RGB for better visualization
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Apply Gaussian blur
        #hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Define range of green, red and orange colors in HSV
        lower_all = np.array([0, 185, 0]) #([0, 69, 0])
        upper_all = np.array([179, 255, 100])

        # Define range for STOP sign and create mask for it
        lower_STOP = np.array([0, 5, 46]) #W([63, 139, 0]) R([72, 193, 163])
        upper_STOP = np.array([33, 61, 75]) #W([176, 173, 135]) R([74, 254, 255])
        mask_STOP = cv2.inRange(imageRGB, lower_STOP, upper_STOP)


        # Threshold the HSV image to get only selected colors
        mask = cv2.inRange(hsv, lower_all, upper_all)
        blur = cv2.medianBlur(mask, 9)
        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
        
        # Erode image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        eroded  = cv2.erode(sharpen, kernel, iterations = 2)

        # Dilate image (Desuse)
        #dilated = cv2.dilate(eroded, kernel, iterations=2)

        # # Perform Canny edge detection (Desuse)
        # edges = cv2.Canny(mask, 50, 150)

        # Create publish message 
        cmd_publish = Twist()

        ############ Find STOP signal #####################
        # Find contours of mask_STOP, detect total area and show it in console 
        total_STOP_area = -1
        contours_STOP, _ = cv2.findContours(mask_STOP, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour_STOP in contours_STOP:
            total_STOP_area = total_STOP_area + cv2.contourArea(contour_STOP)
        print("stop area value = ", total_STOP_area)
        
        # Find contours of green objects
        contours, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a blank image with the same dimensions as the original image
        contour_image = np.zeros_like(image)

        # Draw contours on the blank image
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 1)

        # Convert contour image to 8-bit depth
        #contour = cv2.convertScaleAbs(contour)
        
        # Image dimensions
        height, width = image.shape[:2]

        ## Now trying to move ##

        # Initialize variables to store center of the largest contour
        largest_x = None
        largest_y = None
        max_area = -1
        max_radius = -1
        
        # Analize area of every contour
        for contour in contours:

            # Get area of contour
            area = cv2.contourArea(contour)

            if area > 50: #500 and area < 6000:

                # Keep track of largest shape (area-wise)
                if area > max_area:
                    max_area = area
                    # (largest_x, largest_y), radius = cv2.minEnclosingCircle(contour)
                    # largest_contour = contour

                # Fit a circle to each shape and a circle to its center
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(imageRGB, center, radius, (0, 255, 0), 2)  # Circle in centroid
                cv2.circle(imageRGB, center, 10, (0, 255, 0), -1)  # Circle in center  

                # Keep track of largest shape (radious-wise) and inside the margins
                if radius > max_radius and margin < int(x) < image.shape[1] - margin and margin < int(y) < image.shape[0] - margin : 
                    max_radius = radius
                    largest_x = int(x)
                    largest_y = int(y)
                    largest_contour = contour

        ## Attempt to follow largest shape if it exists

        if largest_x == None or largest_y == None:# or max_area > 80000:

            print("No shape found :c", max_area)
            cmd_publish.linear.x = -0.15 # Move backwards
            cmd_publish.linear.y = 0.0
            cmd_publish.linear.z = 0.0
            cmd_publish.angular.x = 0.0
            cmd_publish.angular.y = 0.0
            cmd_publish.angular.z = 0.2
            self.publisher_.publish(cmd_publish)
   
        else:
            
            ## Is the border too close?

            # Compute the bounding rectangle of the largest contour and its center
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w / 2
            center_y = y + h / 2

            # Check if the center is close to any of the borders within [Margin] pixels
            if (center_x <= margin or center_x >= width - margin or center_y <= margin or center_y >= height - margin):
                print("Center of the contour is close to a border, trying to escape", max_area)
                cmd_publish.linear.x = -0.15 # Move backwards
                cmd_publish.linear.y = 0.0
                cmd_publish.linear.z = 0.0
                cmd_publish.angular.x = 0.0
                cmd_publish.angular.y = 0.0
                cmd_publish.angular.z = 0.0
                self.publisher_.publish(cmd_publish)
                time.sleep(0.1)

            else:

                # Draw cyan vertices in polygon
                for vertex in largest_contour:
                    x, y = vertex[0]
                    cv2.circle(imageRGB, (x, y), 5, (0, 255, 255), -1)
            
                # Draw a purple circle in the center of the biggest contour
                cv2.circle(imageRGB, (int(largest_x), int(largest_y)), 10, (255, 0, 255), -1)

                # Follow biggest contour horizontally
                center_x_image = image.shape[1] // 2
                center_y_image = image.shape[0] // 2
                # Draw a rectangle in the target area and outside the allowed area
                cv2.rectangle(imageRGB, (center_x_image - allowed_centering_error, center_y_image - allowed_centering_error), (center_x_image + allowed_centering_error, center_y_image + allowed_centering_error), (255, 0, 255), 2)
                cv2.rectangle(imageRGB, (margin, margin), (image.shape[1] - margin, image.shape[0] - margin), (126, 0, 255), 2)
                if largest_x > center_x_image + allowed_centering_error:
                    print("Turning right...",max_area)
                    cmd_publish.linear.x = 0.0
                    cmd_publish.linear.y = 0.0
                    cmd_publish.linear.z = 0.0
                    cmd_publish.angular.x = 0.0
                    cmd_publish.angular.y = 0.0
                    cmd_publish.angular.z = -0.02 # Turn right
                    self.publisher_.publish(cmd_publish)
                    
                elif largest_x < center_x_image - allowed_centering_error:
                    print("Turning left...",max_area)
                    cmd_publish.linear.x = 0.0
                    cmd_publish.linear.y = 0.0
                    cmd_publish.linear.z = 0.0
                    cmd_publish.angular.x = 0.0
                    cmd_publish.angular.y = 0.0
                    cmd_publish.angular.z = 0.02  # Turn left
                    self.publisher_.publish(cmd_publish)

                else:
                    print("Centered horizontally")
                    cmd_publish.angular.z = 0.0  # Stop steering
                    self.publisher_.publish(cmd_publish)

                # Follow biggest contour vertically
                    if largest_y < center_y_image - allowed_centering_error:
                        print("Moving up...", max_area)
                        cmd_publish.linear.x = 0.0
                        cmd_publish.linear.y = 0.0
                        cmd_publish.linear.z = 0.02  # Move up
                        cmd_publish.angular.x = 0.0
                        cmd_publish.angular.y = 0.0
                        cmd_publish.angular.z = 0.0
                        self.publisher_.publish(cmd_publish)
                    elif largest_y > center_y_image + allowed_centering_error:
                        print("Moving down...", max_area)
                        cmd_publish.linear.x = 0.0
                        cmd_publish.linear.y = 0.0
                        cmd_publish.linear.z = -0.02  # Move down
                        cmd_publish.angular.x = 0.0
                        cmd_publish.angular.y = 0.0
                        cmd_publish.angular.z = 0.0
                        self.publisher_.publish(cmd_publish)
                    else:
                        if max_area > 70000:
                            print("##################### COLLISION AVOIDANCE ######################",max_area)
                            cmd_publish.linear.x = -0.01 # Move backwards
                            cmd_publish.linear.y = 0.0
                            cmd_publish.linear.z = 0.0
                            cmd_publish.angular.x = 0.0
                            cmd_publish.angular.y = 0.0
                            cmd_publish.angular.z = 0.0
                            self.publisher_.publish(cmd_publish)
                            time.sleep(0.05)
                        else:
                            if total_STOP_area > land_at_STOP:
                                print("STOP is visible", total_STOP_area)

                                ################################# ATTEMPT TO LAND #############################################
                                cmd_publish.linear.x = 0.035  # Move forward
                                cmd_publish.linear.y = 0.0
                                cmd_publish.linear.z = 0.0
                                cmd_publish.angular.x = 0.0
                                cmd_publish.angular.y = 0.0
                                cmd_publish.angular.z = 0.0
                                self.publisher_.publish(cmd_publish)
                                print("Landing...")
                                time.sleep(2.4)
                                self.cli = self.node.create_client(TelloAction, '/drone1/tello_action') # Create client
                                self.publisher_ = self.node.create_publisher(Twist, '/drone1/cmd_vel', 10) # Create publisher for sending Twist messages
                                # Wait for the service to become available
                                while not self.cli.wait_for_service(timeout_sec=1.0):
                                    self.get_logger().info('Service not available yet, waiting...')  
                                self.req = TelloAction.Request() # Create request message for takeoff command
                                self.req.cmd = 'land'  # Set the command to 'LAND'
                                self.future = self.cli.call_async(self.req)
                                print("Landed.")
                                rclpy.shutdown()
                                #time.sleep(100)

                            print("Drone is centered. Moving forward... :)",max_area)
                            time.sleep(0.05)
                            cmd_publish.linear.x = 0.6  # Move forward
                            cmd_publish.linear.y = 0.0
                            cmd_publish.linear.z = 0.0   # Stop leveling
                            cmd_publish.angular.x = 0.0
                            cmd_publish.angular.y = 0.0
                            cmd_publish.angular.z = 0.0
                            self.publisher_.publish(cmd_publish)

        # # Publish movement commands
        # self.publisher_.publish(cmd_publish)

        ## Display the images

        cv2.imshow('imageRGB', imageRGB)
        cv2.imshow('mask', mask)
        cv2.imshow('blur', blur)
        cv2.imshow('sharpen', sharpen)
        cv2.imshow('eroded', eroded)
        cv2.imshow('contour_image', contour_image)
        cv2.imshow('mask_STOP', mask_STOP)

        cv2.moveWindow('imageRGB', 0, 0)
        cv2.moveWindow('mask', 525, 0)
        cv2.moveWindow('blur', 980, 0)
        cv2.moveWindow('sharpen', 0, 420)
        cv2.moveWindow('eroded', 525, 420) 
        cv2.moveWindow('contour_image', 980, 420)
        cv2.moveWindow('mask_STOP', 1400, 0)

        cv2.waitKey(1)

        # Add a little pause for me to see what is happening
        #time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    Open_Project_Aerial_Robotics = OpenProjectAerialRobotics()
    rclpy.spin(Open_Project_Aerial_Robotics.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()