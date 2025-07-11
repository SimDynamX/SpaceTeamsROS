#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import MoveLeft
import time


class MoveLeftServer(Node):
    def __init__(self):
        super().__init__('move_left_server')
        self.srv = self.create_service(MoveLeft, 'move_left', self.move_left_callback)
        self.get_logger().info('MoveLeft service server is ready.')

    def move_left_callback(self, request, response):
        self.get_logger().info(f'Incoming request: distance={request.distance}, direction_type={request.direction_type}')
        
        # Simulate some processing time
        time.sleep(1.0)
        
        # Simple logic for the service
        if request.distance > 0.0:
            response.success = True
            response.message = f'Successfully moved left {request.distance} units with {request.direction_type} direction'
            response.actual_distance = request.distance
            self.get_logger().info(f'Service completed successfully: moved {request.distance} units')
        else:
            response.success = False
            response.message = 'Invalid distance: must be greater than 0'
            response.actual_distance = 0.0
            self.get_logger().warn('Service failed: invalid distance')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    move_left_server = MoveLeftServer()
    
    try:
        rclpy.spin(move_left_server)
    except KeyboardInterrupt:
        pass
    
    move_left_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
