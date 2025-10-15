

##### OLD CODE BELOW - KEEP FOR REFERENCE #####
'''
from urllib import response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
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
            CompressedImage,
            'camera/image_raw/compressed',
            self.image_callback,
            10
        )

    def image_callback(self, msg: CompressedImage):
        """Callback function to handle incoming image messages"""
        # Process and display the received image
        image_msg = self.compressed_to_image(msg)
        self.view_image(image_msg)

    def compressed_to_image(self, compressed_msg: CompressedImage) -> Image:
        """Convert CompressedImage message to Image message"""
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        return image_msg

    def view_image(self, image_data: Image):
        """Process and display the image data as a video stream"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')

            # Log image information (optional)
            height, width, channels = cv_image.shape

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

'''






import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import cv2
import numpy as np
import time
from typing import Dict, Optional
from dataclasses import dataclass
import subprocess

@dataclass
class TopicConfig:
    name: str
    qos_history_depth: int = 5  # Keep more messages in history
    qos_reliability: int = rclpy.qos.ReliabilityPolicy.BEST_EFFORT  # Best effort for real-time
    qos_durability: int = rclpy.qos.DurabilityPolicy.VOLATILE  # No need to store

class ImageSubscriber(Node):
    def __init__(self, 
                rgb_topic: Optional[TopicConfig] = None,
                depth_topic: Optional[TopicConfig] = None):
        """Initialize the image subscriber node with configurable topics.
        
        Args:
            rgb_topic: Configuration for RGB image topic
            depth_topic: Configuration for depth image topic
        """
        # Initialize the ROS node first
        super().__init__('image_client')
        

        self.get_logger().info('[ST] Image client is starting...')
        
        # Initialize the CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        #TODO BREAKING create subscription to pass along image messages?
        # self.subscription = self.create_subscription(
        #     CompressedImage,
        #     'camera/image_raw/compressed',
        #     self.image_callback,
        #     10
        # )

        # Store constructor parameters
        self._rgb_topic_config = rgb_topic or TopicConfig(name='camera/image_raw')
        self._depth_topic_config = depth_topic or TopicConfig(name='camera/depth/image_raw')
        
        # Initialize core instance variables first
        self.bridge = CvBridge()
        self.connected = False
        
        # Initialize tracking dictionaries
        self.frames_count = {'RGB': 0, 'DEPTH': 0}
        self.last_time = {'RGB': time.time(), 'DEPTH': time.time()}
        
        # Initialize the publishers dictionary after super().__init__()
        self.my_publishers = {}
        
        # Setup ZMQ subscriber
        self._setup_zmq_subscriber()
        
        # Setup publishers for different image types
        self._setup_publishers()
        
        # Create timer for receiving images at maximum possible rate
        period = 0.0001  # 10kHz - effectively as fast as possible
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info("Image subscriber started - Processing at maximum rate")
        
    def _setup_zmq_subscriber(self) -> None:
        """Setup and configure ZMQ subscriber socket."""
        self._zmq_context = zmq.Context()
        self._zmq_context.setsockopt(zmq.MAX_SOCKETS, 1)
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        
        # Configure socket for high-performance streaming
        self._zmq_socket.setsockopt(zmq.RCVHWM, 2)  # Small HWM to prevent memory growth
        self._zmq_socket.setsockopt(zmq.LINGER, 0)  # Don't wait on close
        self._zmq_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep latest message
        self._zmq_socket.setsockopt(zmq.TCP_KEEPALIVE, 1)  # Keep connection alive
        self._zmq_socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 120)  # Faster keepalive
        self._zmq_socket.setsockopt(zmq.RCVBUF, 2*1024*1024)  # 2MB receive buffer
        self._zmq_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second receive timeout
        
        # Subscribe to all messages (we'll filter them after receiving)
        self._zmq_socket.setsockopt(zmq.SUBSCRIBE, b"")
        
        # Connect to the Windows host
        windows_ip = self._get_windows_ip()
        connect_address = f"tcp://{windows_ip}:5555"
        self.get_logger().info(f"Connecting to publisher at {connect_address}")
        try:
            self._zmq_socket.connect(connect_address)
            self.get_logger().info(f"Successfully connected to {connect_address}")
            self.connected = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

    def _get_windows_ip(self) -> str:
        """Get the IP address of the Windows host from WSL."""
        try:
            result = subprocess.run(['cat', '/etc/resolv.conf'], capture_output=True, text=True)
            nameserver_line = [line for line in result.stdout.split('\n') if 'nameserver' in line][0]
            return nameserver_line.split()[1]
        except Exception as e:
            self.get_logger().error(f"Failed to get Windows IP: {str(e)}")
            return "172.17.0.1"  # Common WSL2 default

    def _setup_publishers(self) -> None:
        """Setup ROS publishers for different image types with optimized QoS settings."""
        # Setup RGB image publisher if configured
        if self._rgb_topic_config:
            self.my_publishers['RGB'] = self.create_publisher(
                Image,
                self._rgb_topic_config.name,
                qos_profile=self._create_qos_profile(self._rgb_topic_config)
            )
            
        # Setup depth image publisher if configured
        if self._depth_topic_config:
            self.my_publishers['DEPTH'] = self.create_publisher(
                Image,
                self._depth_topic_config.name,
                qos_profile=self._create_qos_profile(self._depth_topic_config)
            )
        
    def _create_qos_profile(self, topic_config: TopicConfig) -> rclpy.qos.QoSProfile:
        """Create QoS profile for image publishers."""
        return rclpy.qos.QoSProfile(
            reliability=topic_config.qos_reliability,
            durability=topic_config.qos_durability,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=topic_config.qos_history_depth,
            # Tighter deadlines for real-time performance
            deadline=rclpy.duration.Duration(seconds=1/60.0),
            lifespan=rclpy.duration.Duration(seconds=1/30.0),
            liveliness=rclpy.qos.LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=rclpy.duration.Duration(seconds=0.1)  # Faster liveliness checks
        )

    def _process_frame(self, frame_data: bytes) -> None:
        """Process received frame data and publish to appropriate ROS topic."""
        frame_type = None
        try:
            # Find all # positions in the first 50 bytes (header should be in there)
            hash_positions = []
            for i in range(min(50, len(frame_data))):
                if frame_data[i:i+1] == b'#':
                    hash_positions.append(i)

            if len(hash_positions) < 4:  # Need at least 4 # for complete header
                raise ValueError("Could not find complete header pattern")

            # Get the header section (up to the fourth #)
            header_end = hash_positions[3]  # Fourth # mark
            header = frame_data[:header_end].decode('ascii')
            
            # Parse the header parts
            parts = header.split('#')
            if len(parts) < 4:  # Should have frame_type, height, width, channels
                raise ValueError(f"Invalid header format: {parts}")
                
            frame_type = parts[0]
            height = int(parts[1])
            width = int(parts[2])
            channels = int(parts[3])
            
            # Extract image data (everything after the last #)
            img_data = frame_data[header_end + 1:]
            
            # Verify we have enough data
            expected_size = height * width * channels
            if len(img_data) != expected_size:
                raise ValueError(f"Image data size mismatch. Expected {expected_size} bytes, got {len(img_data)} bytes")
            
            # Convert to numpy array
            frame = np.frombuffer(img_data, dtype=np.uint8).reshape((height, width, channels))
            
            # Convert to ROS message
            if frame_type in self.my_publishers:
                # Pre-allocate header for optimization
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_type.lower() + "_camera"
                self.my_publishers[frame_type].publish(msg)
                
                # Update statistics (less frequently to reduce overhead)
                self.frames_count[frame_type] += 1
                current_time = time.time()
                if current_time - self.last_time[frame_type] > 2.0:  # Update every 2 seconds
                    fps = self.frames_count[frame_type] / (current_time - self.last_time[frame_type])
                    self.get_logger().info(f"{frame_type} FPS: {fps:.1f}")
                    self.frames_count[frame_type] = 0
                    self.last_time[frame_type] = current_time
                    
        except Exception as e:
            error_msg = f"Error processing frame: {str(e)}"
            if frame_type:
                error_msg = f"Error processing {frame_type} frame: {str(e)}"
            self.get_logger().error(error_msg)

    def timer_callback(self):
        """Timer callback to receive and process frames as fast as possible."""
        if not self.connected:
            return
            
        try:
            # Process messages as fast as possible
            for _ in range(100):  # Process up to 100 messages per callback
                try:
                    frame_data = self._zmq_socket.recv(flags=zmq.NOBLOCK)
                    self._process_frame(frame_data)
                except zmq.Again:
                    return  # No more messages
                    
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

    
    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
