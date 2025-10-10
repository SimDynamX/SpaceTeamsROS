#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import String, Float
from geometry_msgs.msg import Point, Quaternion
import math
import time
from space_teams_python.transformations import *


class RoverController(Node):
    def __init__(self):
        super().__init__('RoverController')
        # Service clients
        self.logger_client = self.create_client(String, 'log_message')
        self.steer_client = self.create_client(Float, 'Steer')
        self.accelerator_client = self.create_client(Float, 'Accelerator')
        self.reverse_client = self.create_client(Float, 'Reverse')
        self.brake_client = self.create_client(Float, 'Brake')
        self.core_sampling_client = self.create_client(Float, 'CoreSample')

        # Topic subscriptions
        self.current_location_marsFrame = None
        self.current_velocity_marsFrame = None
        self.current_rotation_marsFrame = None
        self.current_location_localFrame = None
        self.current_velocity_localFrame = None
        self.current_rotation_localFrame = None
        self.state = "Driving"

        self.create_subscription(Point, 'LocationMarsFrame', self.location_marsFrame_callback, 10)
        self.create_subscription(Point, 'VelocityMarsFrame', self.velocity_marsFrame_callback, 10)
        self.create_subscription(Quaternion, 'RotationMarsFrame', self.rotation_marsFrame_callback, 10)
        self.create_subscription(Point, 'LocationLocalFrame', self.location_localFrame_callback, 10)
        self.create_subscription(Point, 'VelocityLocalFrame', self.velocity_localFrame_callback, 10)
        self.create_subscription(Quaternion, 'RotationLocalFrame', self.rotation_localFrame_callback, 10)

        self.create_subscription(Point, 'CoreSamplingComplete', self.core_sampling_complete_callback, 1)

        # Control state
        self.target_loc_localFrame = None
        self.tolerance = 5.0  # meters
        self.max_speed = 0.5
        self.navigation_active = False
        self.navigation_iterations = 0
        self.initial_move_end_time = None
        self.initial_move_done = False

        # Waypoints
        self.waypoints = None
        self.current_waypoint_idx = None

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Rover controller is ready.')

    def location_marsFrame_callback(self, msg):
        self.current_location_marsFrame = msg
    
    def velocity_marsFrame_callback(self, msg):
        self.current_velocity_marsFrame = msg

    def rotation_marsFrame_callback(self, msg):
        self.current_rotation_marsFrame = msg

    def location_localFrame_callback(self, msg):
        self.current_location_localFrame = msg

    def velocity_localFrame_callback(self, msg):
        self.current_velocity_localFrame = msg

    def rotation_localFrame_callback(self, msg):
        self.current_rotation_localFrame = msg
    
    def core_sampling_complete_callback(self, msg):
        self.state = "Driving"

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

    def send_reverse_command(self, reverse_value):
        request = Float.Request()
        request.data = max(0.0, min(1.0, reverse_value))
        return self.reverse_client.call_async(request)

    def send_brake_command(self, brake_value):
        request = Float.Request()
        request.data = max(0.0, min(1.0, brake_value))
        return self.brake_client.call_async(request)
    
    def send_core_sampling_command(self):
        self.state = "Sampling"
        request = Float.Request()
        request.data = 0.0
        return self.core_sampling_client.call_async(request)
    
    def calculate_direction_to_target(self, current_loc_localFrame: npt.NDArray, 
                                      target_loc_localFrame: npt.NDArray) -> npt.NDArray:
        return normalize(target_loc_localFrame - current_loc_localFrame)
    
    def calculate_error_angle_sign(self, vec1: npt.NDArray, vec2: npt.NDArray) -> float:
        return 1.0 if np.dot(np.cross(vec1, vec2), np.array([0.0, 0.0, 1.0])) > 0.0 else -1.0
    
    def error_angle_arctan(self, vec1, vec2):
        up = normalize(np.cross(np.cross(vec2, vec1), vec2))
        x = np.dot(vec1, vec2)
        y = np.dot(vec1, up)
        return np.arctan2(y, x)
    
    def calculate_pointing_error_angle(self, current_loc_localFrame: npt.NDArray, 
                                       target_loc_localFrame: npt.NDArray, current_rot_localFrame: Quat) -> float:
        m = current_rot_localFrame.to_matrix()
        forward = m[:, 0]
        forward = normalize(np.array([forward[0], forward[1], 0.0]))

        target_direction = self.calculate_direction_to_target(current_loc_localFrame, target_loc_localFrame)
        target_direction = normalize(np.array([target_direction[0], target_direction[1], 0.0]))

        # error_angle = self.error_angle_arctan(forward, target_direction)

        error_angle = np.arccos(np.dot(forward, target_direction))
        error_angle_dir = self.calculate_error_angle_sign(target_direction, forward)
        return error_angle_dir * error_angle

    def calculate_distance_to_target(self, current_loc_localFrame: npt.NDArray, target_loc_localFrame: npt.NDArray):
        return np.linalg.norm(target_loc_localFrame - current_loc_localFrame)

    def calculate_speed_difference(self, current_vel_localFrame: npt.NDArray, target_speed_kph: float) -> float:
        return mps_to_kph(kph_to_mps(target_speed_kph) - np.linalg.norm(current_vel_localFrame))

    def start_navigation(self, target_loc_localFrame):
        self.target_loc_localFrame = target_loc_localFrame
        self.navigation_active = True
        self.navigation_iterations = 0
        self.initial_move_done = False
        self.initial_move_end_time = time.time() + 10.0
        self.log_message(f"Starting navigation to target: ({target_loc_localFrame[0]:.2f}, {target_loc_localFrame[1]:.2f})")
        self.send_accelerator_command(0.2)

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
        if self.current_location_localFrame is None or self.current_rotation_localFrame is None:
            self.get_logger().info("Waiting for location/rotation update...")
            return

        # Get location
        current_x = float(self.current_location_localFrame.x)
        current_y = float(self.current_location_localFrame.y)
        current_z = float(self.current_location_localFrame.z)
        current_loc_localFrame = np.array([current_x, current_y, current_z])

        # Get velocity
        current_vx = float(self.current_velocity_localFrame.x)
        current_vy = float(self.current_velocity_localFrame.y)
        current_vz = float(self.current_velocity_localFrame.z)
        current_vel_localFrame = np.array([current_vx, current_vy, current_vz])

        # Get rotation
        qx = float(self.current_rotation_localFrame.x)
        qy = float(self.current_rotation_localFrame.y)
        qz = float(self.current_rotation_localFrame.z)
        qw = float(self.current_rotation_localFrame.w)
        current_rot_localFrame = Quat(qw, qx, qy, qz)

        # Distance to target
        distance = self.calculate_distance_to_target(current_loc_localFrame, self.target_loc_localFrame)
        if distance < self.tolerance:
            self.send_brake_command(1.0)
            self.send_steer_command(0.0)
            self.send_accelerator_command(0.0)
            self.log_message(f"Target reached! Beginning core sampling at position: ({current_x:.2f}, {current_y:.2f})")
            self.send_core_sampling_command()

            if self.current_waypoint_idx == len(self.waypoints) - 1:
                self.navigation_active = False
                self.log_message("Navigation complete: all waypoints reached and all core samples collected.")
            else:
                self.current_waypoint_idx += 1
                self.target_loc_localFrame = self.waypoints[self.current_waypoint_idx]
                next_loc = f"({self.target_loc_localFrame[0]:.2f}, {self.target_loc_localFrame[1]:.2f})"
                self.log_message(f"After sampling, moving to next waypoint at: {next_loc}")
            return
        
        # Velocity error
        speed_limit_kph = 15.0
        speed_diff_kph = self.calculate_speed_difference(current_vel_localFrame, speed_limit_kph)  # target - current
        accel_factor = remap_clamp(0.0, speed_limit_kph, 0.0, 1.0, speed_diff_kph)  # 1 if not moving, 0 if too fast
        brake_factor = 1.0 - remap_clamp(-speed_limit_kph, 0.0, 0.0, 1.0, speed_diff_kph)  # 0 if <= speed limit, 1 if 2x over

        # Heading error
        db_heading = np.deg2rad(3.0)  # deadband for heading alignment
        heading_error = self.calculate_pointing_error_angle(current_loc_localFrame, self.target_loc_localFrame, 
                                                            current_rot_localFrame)
        
        # Steering
        steer_command = remap_clamp(-0.25 * np.pi, 0.25 * np.pi, -1.0, 1.0, heading_error)
        if abs(heading_error) < db_heading:
            steer_command = 0.0
        steer_gain = 1.0
        actual_steer_command = -steer_gain * steer_command
        
        # Acceleration
        accel_gain = 1.0
        accel_command = accel_gain * remap_clamp(0.0, 1.0, accel_factor, accel_factor * 0.5, abs(steer_command))

        # Braking
        # If brake, brake_command > 0.5 results in braking (i.e., boolean behavior)
        # If reverse, float value between 0 and 1 is passed, acts as a gradual deceleration
        brake_gain = 1.0
        brake_command = brake_gain * brake_factor

        self.send_steer_command(actual_steer_command)
        self.send_accelerator_command(accel_command)
        self.send_reverse_command(brake_command)  # Send brake command as a float (reverse)
        self.send_brake_command(0.0)
        # self.send_brake_command(brake_command)  # Send brake command as a bool

        # Print commands for debugging:
        # if self.navigation_iterations % 10 == 0:
        #     self.log_message(
        #         f"Position: ({current_x:.2f}, {current_y:.2f}), "
        #         f"Distance: {distance:.2f}, "
        #         f"Heading error: {math.degrees(heading_error):.1f} deg, "
        #         f"Steer: {steer_command:.2f}, "
        #         f"Accel: {accel_command:.2f}"
        #     )
        self.navigation_iterations += 1


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()

    # Test waypoint:
    # waypoint_marsframe = np.array([2193073.87847882, 743984.99629174, -2485667.65565136])
    # waypoint_localframe = np.array([22.0285988, 60.41062071, -4.50449595])

    # Test multiple waypoints:
    waypoints_localFrame = [
        np.array([22.0285988, 60.41062071, -4.50449595]),
        np.array([-404.64432933, 3.8770865, -18.18814408])
    ]

    # Wait for initial location and rotation
    while rclpy.ok():
        if rover_controller.current_location_localFrame is not None and rover_controller.current_rotation_localFrame is not None:
            break
        rover_controller.get_logger().info('Waiting for initial location and rotation...')
        rclpy.spin_once(rover_controller, timeout_sec=0.5)

    current_x = rover_controller.current_location_localFrame.x
    current_y = rover_controller.current_location_localFrame.y

    rover_controller.waypoints = waypoints_localFrame
    rover_controller.current_waypoint_idx = 0

    rover_controller.log_message(
        f"Starting navigation: moving from ({current_x:.2f}, {current_y:.2f}) to ({waypoints_localFrame[0][0]:.2f}, {waypoints_localFrame[0][1]:.2f})"
    )
    
    rover_controller.start_navigation(waypoints_localFrame[0])

    try:
        rclpy.spin(rover_controller)
    finally:
        rover_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()