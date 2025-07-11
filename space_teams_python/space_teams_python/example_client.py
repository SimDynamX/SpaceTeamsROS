#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import StringService
import sys


class LoggerClient(Node):
    def __init__(self):
        super().__init__('LoggerClient')
        self.cli = self.create_client(StringService, 'log_message')
        
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('spaceteams logger service client is ready.')

    def send_request(self, data):
        request = StringService.Request()
        request.data = data
        
        future = self.cli.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)

    logger_client = LoggerClient()

    # Default values
    data = "logging message for spaceTeams!"
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        try:
            data = float(sys.argv[1])
        except ValueError:
            logger_client.get_logger().error('Invalid distance value. Using default: 0.5')
    
    # Send the request
    future = logger_client.send_request(data)
    
    # Wait for the response
    rclpy.spin_until_future_complete(logger_client, future)

    if future.result() is not None:
        response = future.result()
        logger_client.get_logger().info(f'Response: success={response.success}')
    else:
        logger_client.get_logger().error('Service call failed')
    
    logger_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
