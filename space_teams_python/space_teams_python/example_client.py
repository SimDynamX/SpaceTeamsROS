#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import String, Float
from geometry_msgs.msg import Point, Quaternion
import math
import time

class RoverController(Node):
    def __init__(self):
        super().__init__('RoverController')
        # Service clients
        self.logger_client = self.create_client(String, 'log_message')
        self.steer_client = self.create_client(Float, 'Steer')
        self.accelerator_client = self.create_client(Float, 'Accelerator')
        self.brake_client = self.create_client(Float, 'Brake')

        # Topic subscriptions
        self.current_location = None
        self.current_rotation = None
        self.create_subscription(Point, 'Location', self.location_callback, 10)
        self.create_subscription(Quaternion, 'Rotation', self.rotation_callback, 10)

        # Control state
        self.target_x = None
        self.target_y = None
        self.tolerance = 5.0
        self.max_speed = 0.5
        self.navigation_active = False
        self.navigation_iterations = 0
        self.max_iterations = 1000
        self.initial_move_end_time = None
        self.initial_move_done = False

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Rover controller is ready.')

    def location_callback(self, msg):
        self.current_location = msg

    def rotation_callback(self, msg):
        self.current_rotation = msg

    def log_message(self, message):
        request = String.Request()
        request.data = message
        future = self.logger_client.call_async(request)
        return future

    def send_steer_command(self, steer_value):
        request = Float.Request()
        request.data = max(-1.0, min(1.0, steer_value))
        return self.steer_client.call_async(request)

    def send_accelerator_command(self, accel_value):
        request = Float.Request()
        request.data = max(0.0, min(1.0, accel_value))
        return self.accelerator_client.call_async(request)

    def send_brake_command(self, brake_value):
        request = Float.Request()
        request.data = max(0.0, min(1.0, brake_value))
        return self.brake_client.call_async(request)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_bearing_to_target(self, current_x, current_y, target_x, target_y):
        dx = target_x - current_x
        dy = target_y - current_y
        return math.atan2(dy, dx)

    def calculate_distance_to_target(self, current_x, current_y, target_x, target_y):
        dx = target_x - current_x
        dy = target_y - current_y
        return math.sqrt(dx*dx + dy*dy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def start_navigation(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.navigation_active = True
        self.navigation_iterations = 0
        self.initial_move_done = False
        self.initial_move_end_time = time.time() + 4.0
        self.log_message(f"Starting navigation to target: ({target_x:.2f}, {target_y:.2f})")
        self.send_accelerator_command(0.5)

    def timer_callback(self):
        if not self.navigation_active:
            return

        # Initial move forward for 4 seconds
        if not self.initial_move_done and self.initial_move_end_time is not None:
            if time.time() < self.initial_move_end_time:
                return
            self.send_accelerator_command(0.0)
            self.initial_move_done = True
        # Navigation logic
        if self.navigation_iterations >= self.max_iterations:
            self.log_message("Maximum iterations reached, stopping navigation")
            self.navigation_active = False
            return

        if self.current_location is None or self.current_rotation is None:
            self.get_logger().info("Waiting for location/rotation update...")
            return

        current_x = self.current_location.x
        current_y = self.current_location.y
        current_z = self.current_location.z
        qx = self.current_rotation.x
        qy = self.current_rotation.y
        qz = self.current_rotation.z
        qw = self.current_rotation.w

        distance = self.calculate_distance_to_target(current_x, current_y, self.target_x, self.target_y)
        if distance < self.tolerance:
            self.get_logger().info("Target reached. Sending stop commands.")
            self.send_brake_command(1.0)
            self.send_steer_command(0.0)
            self.send_accelerator_command(0.0)
            self.log_message(f"Target reached! Final position: ({current_x:.2f}, {current_y:.2f})")
            self.navigation_active = False
            return

        current_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        target_bearing = self.calculate_bearing_to_target(current_x, current_y, self.target_x, self.target_y)
        heading_error = self.normalize_angle(target_bearing - current_yaw)
        steer_gain = 2.0
        steer_command = max(-1.0, min(1.0, steer_gain * heading_error))
        distance_factor = min(1.0, distance / 50.0)
        heading_factor = max(0.3, 1.0 - abs(heading_error) / math.pi)
        accel_command = self.max_speed * distance_factor * heading_factor

        self.send_steer_command(steer_command)
        self.send_accelerator_command(accel_command)
        self.send_brake_command(0.0)

        if self.navigation_iterations % 10 == 0:
            self.log_message(
                f"Position: ({current_x:.2f}, {current_y:.2f}), "
                f"Distance: {distance:.2f}, "
                f"Heading error: {math.degrees(heading_error):.1f}°, "
                f"Steer: {steer_command:.2f}, "
                f"Accel: {accel_command:.2f}"
            )
        self.navigation_iterations += 1

def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()

    # Wait for initial location and rotation
    while rclpy.ok():
        if rover_controller.current_location is not None and rover_controller.current_rotation is not None:
            break
        rover_controller.get_logger().info('Waiting for initial location and rotation...')
        rclpy.spin_once(rover_controller, timeout_sec=0.5)

    current_x = rover_controller.current_location.x
    current_y = rover_controller.current_location.y
    current_z = rover_controller.current_location.z

    target_x = current_x + 100.0
    target_y = current_y + 100.0
    rover_controller.log_message(
        f"Starting mission: Moving from ({current_x:.2f}, {current_y:.2f}) to ({target_x:.2f}, {target_y:.2f})"
    )
    rover_controller.start_navigation(target_x, target_y)

    try:
        rclpy.spin(rover_controller)
    finally:
        rover_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()