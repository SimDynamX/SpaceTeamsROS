#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import String, Vector3d, Float, Quaternion
import sys
import math
import time


class RoverController(Node):
    def __init__(self):
        super().__init__('RoverController')
        
        # Create service clients
        self.logger_client = self.create_client(String, 'log_message')
        self.steer_client = self.create_client(Float, 'Steer')
        self.accelerator_client = self.create_client(Float, 'Accelerator')
        self.brake_client = self.create_client(Float, 'Brake')
        self.location_client = self.create_client(Vector3d, 'GetLocation')
        self.rotation_client = self.create_client(Quaternion, 'GetRotation')
        
        # Wait for all services to be available
        services = [
            (self.logger_client, 'log_message'),
            (self.steer_client, 'Steer'),
            (self.accelerator_client, 'Accelerator'),
            (self.brake_client, 'Brake'),
            (self.location_client, 'GetLocation'),
            (self.rotation_client, 'GetRotation')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {service_name} not available, waiting again...')
        
        self.get_logger().info('Rover controller is ready.')
        
        # Control parameters
        self.target_x = None
        self.target_y = None
        self.tolerance = 5.0  # Distance tolerance to consider target reached
        self.max_speed = 0.5  # Maximum acceleration
        self.max_steer = 1.0  # Maximum steering

    def log_message(self, message):
        """Send a log message"""
        request = String.Request()
        request.data = message
        future = self.logger_client.call_async(request)
        return future

    def get_current_location(self):
        """Get the current location of the rover"""
        request = Vector3d.Request()
        future = self.location_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.x, response.y, response.z
        return None, None, None

    def get_current_rotation(self):
        """Get the current rotation of the rover"""
        request = Quaternion.Request()
        future = self.rotation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.x, response.y, response.z, response.w
        return None, None, None, None

    def send_steer_command(self, steer_value):
        """Send steering command (-1 to 1, negative = left, positive = right)"""
        request = Float.Request()
        request.data = max(-1.0, min(1.0, steer_value))  # Clamp to [-1, 1]
        future = self.steer_client.call_async(request)
        return future

    def send_accelerator_command(self, accel_value):
        """Send acceleration command (0 to 1)"""
        request = Float.Request()
        request.data = max(0.0, min(1.0, accel_value))  # Clamp to [0, 1]
        future = self.accelerator_client.call_async(request)
        return future

    def send_brake_command(self, brake_value):
        """Send brake command (0 to 1)"""
        request = Float.Request()
        request.data = max(0.0, min(1.0, brake_value))  # Clamp to [0, 1]
        future = self.brake_client.call_async(request)
        return future

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle in radians"""
        # Calculate yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def calculate_bearing_to_target(self, current_x, current_y, target_x, target_y):
        """Calculate the bearing from current position to target in radians"""
        dx = target_x - current_x
        dy = target_y - current_y
        bearing = math.atan2(dy, dx)
        return bearing

    def calculate_distance_to_target(self, current_x, current_y, target_x, target_y):
        """Calculate the distance to target"""
        dx = target_x - current_x
        dy = target_y - current_y
        return math.sqrt(dx*dx + dy*dy)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_coordinate(self, target_x, target_y):
        """Move the rover to the specified coordinate"""
        self.target_x = target_x
        self.target_y = target_y
        
        # Log the mission start
        future = self.log_message(f"Starting navigation to target: ({target_x:.2f}, {target_y:.2f})")
        rclpy.spin_until_future_complete(self, future)
        
        max_iterations = 1000  # Prevent infinite loops
        iteration = 0
        
        while iteration < max_iterations:
            # Get current position and orientation
            current_x, current_y, current_z = self.get_current_location()
            if current_x is None:
                self.get_logger().error("Failed to get current location")
                break
                
            qx, qy, qz, qw = self.get_current_rotation()
            if qx is None:
                self.get_logger().error("Failed to get current rotation")
                break
            
            # Calculate distance to target
            distance = self.calculate_distance_to_target(current_x, current_y, target_x, target_y)
            
            # Check if we've reached the target
            if distance < self.tolerance:
                # Stop the rover
                self.send_brake_command(1.0)
                rclpy.spin_until_future_complete(self, self.send_steer_command(0.0))
                rclpy.spin_until_future_complete(self, self.send_accelerator_command(0.0))
                
                future = self.log_message(f"Target reached! Final position: ({current_x:.2f}, {current_y:.2f})")
                rclpy.spin_until_future_complete(self, future)
                break
            
            # Calculate current heading and desired bearing
            current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
            target_bearing = self.calculate_bearing_to_target(current_x, current_y, target_x, target_y)
            
            # Calculate steering error
            heading_error = self.normalize_angle(target_bearing - current_yaw)
            
            # Calculate steering command (proportional control)
            steer_gain = 2.0  # Steering sensitivity
            steer_command = max(-1.0, min(1.0, steer_gain * heading_error))
            
            # Calculate acceleration based on distance and heading error
            # Slow down when close to target or when steering correction is large
            distance_factor = min(1.0, distance / 50.0)  # Slow down when closer than 50 units
            heading_factor = max(0.3, 1.0 - abs(heading_error) / math.pi)  # Slow down for large heading errors
            
            accel_command = self.max_speed * distance_factor * heading_factor
            
            # Send commands
            rclpy.spin_until_future_complete(self, self.send_steer_command(steer_command))
            rclpy.spin_until_future_complete(self, self.send_accelerator_command(accel_command))
            rclpy.spin_until_future_complete(self, self.send_brake_command(0.0))
            
            # Log progress every 10 iterations
            if iteration % 10 == 0:
                future = self.log_message(
                    f"Position: ({current_x:.2f}, {current_y:.2f}), "
                    f"Distance: {distance:.2f}, "
                    f"Heading error: {math.degrees(heading_error):.1f}°, "
                    f"Steer: {steer_command:.2f}, "
                    f"Accel: {accel_command:.2f}"
                )
                rclpy.spin_until_future_complete(self, future)
            
            iteration += 1
            time.sleep(0.1)  # Small delay for control loop
        
        if iteration >= max_iterations:
            future = self.log_message("Maximum iterations reached, stopping navigation")
            rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)

    rover_controller = RoverController()

    # Get current location to set target
    current_x, current_y, current_z = rover_controller.get_current_location()
    
    if current_x is not None:
        # Set target as current position + 100 in both x and y directions
        target_x = current_x + 100.0
        target_y = current_y + 100.0
        
        # Log the mission
        future = rover_controller.log_message(
            f"Starting mission: Moving from ({current_x:.2f}, {current_y:.2f}) to ({target_x:.2f}, {target_y:.2f})"
        )
        rclpy.spin_until_future_complete(rover_controller, future)
        
        # Execute the navigation
        rover_controller.move_to_coordinate(target_x, target_y)
    else:
        rover_controller.get_logger().error('Failed to get initial location')
    
    rover_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
