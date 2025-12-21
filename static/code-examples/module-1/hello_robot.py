#!/usr/bin/env python3
"""
Hello Robot - Your First ROS 2 Node
Module 1: The Robotic Nervous System

This is a simple ROS 2 node that publishes a greeting message.
Run this file to test your ROS 2 installation.

Usage:
    ros2 run my_robot_pkg hello_robot
    
Or run directly:
    python3 hello_robot.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloRobot(Node):
    """
    A simple ROS 2 node that demonstrates the pub/sub pattern.
    
    This node publishes a greeting message every second, showing
    the basic structure of a ROS 2 Python node.
    """
    
    def __init__(self):
        # Initialize the node with a name
        super().__init__('hello_robot')
        
        # Create a publisher on the 'greetings' topic
        self.publisher = self.create_publisher(
            String,           # Message type
            'greetings',      # Topic name
            10                # Queue size
        )
        
        # Create a timer that calls our callback every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Counter for tracking messages
        self.count = 0
        
        # Log that we're ready
        self.get_logger().info('🤖 Hello Robot node is running!')
        self.get_logger().info('   Publishing to /greetings topic...')
    
    def timer_callback(self):
        """Called every second to publish a message"""
        # Create the message
        msg = String()
        msg.data = f'Hello, Robot World! Message #{self.count}'
        
        # Publish it
        self.publisher.publish(msg)
        
        # Log to terminal
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment counter
        self.count += 1


def main(args=None):
    """Main function to run the node"""
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create our node
    node = HelloRobot()
    
    try:
        # Spin (keep the node running)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Shutting down Hello Robot...')
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
