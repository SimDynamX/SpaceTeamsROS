
from urllib import response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import math
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')

        self.get_logger().info('Image client is starting...')
        
        # Initialize the CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        # log the average red pixel values, green pixel values, and blue pixel values
        averageRed = np.sum(msg.data[0::3]) / (len(msg.data) // 3)
        averageGreen = np.sum(msg.data[1::3]) / (len(msg.data) // 3)
        averageBlue = np.sum(msg.data[2::3]) / (len(msg.data) // 3)

        self.get_logger().info(f'Average Red: {averageRed}, Average Green: {averageGreen}, Average Blue: {averageBlue}')


        # Process and display the received image
        self.view_image(msg)

    def view_image(self, image_data: Image):
        """Process and display the image data as a video stream"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')

            # Log image information (optional)
            height, width, channels = cv_image.shape
            self.get_logger().info(f'Image received: {width}x{height}, {channels} channels')
            self.get_logger().info(f'Image encoding: {image_data.encoding}')

            # Display the image using OpenCV (window stays open and updates)
            cv2.imshow('Camera Feed', cv_image)
            # Use a small waitKey to keep the window responsive
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # Optional: allow user to close window with 'q'
                cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    image_client = ImageClient()
    rclpy.spin(image_client)
    
    image_client.destroy_node()
    rclpy.shutdown()
