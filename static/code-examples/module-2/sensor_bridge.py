#!/usr/bin/env python3
"""
Sensor Bridge - ROS 2 to Gazebo Sensor Integration
Module 2: The Digital Twin

This node bridges sensor data from Gazebo simulation to ROS 2 topics.
It demonstrates how to:
- Subscribe to simulated sensor data
- Process LiDAR point clouds
- Handle depth camera images
- Read IMU data

Usage:
    ros2 run my_robot_pkg sensor_bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, Image, Imu
from geometry_msgs.msg import Twist
import numpy as np


class SensorBridge(Node):
    """
    Bridge node for integrating Gazebo sensors with ROS 2.
    
    Subscribes to simulated sensor topics and provides
    processed data for navigation and perception.
    """
    
    def __init__(self):
        super().__init__('sensor_bridge')
        
        # QoS profile for sensor data (best effort for simulation)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ----- LIDAR SUBSCRIPTION -----
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            sensor_qos
        )
        
        # ----- DEPTH CAMERA SUBSCRIPTION -----
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            sensor_qos
        )
        
        # ----- IMU SUBSCRIPTION -----
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        # ----- PUBLISHERS -----
        self.obstacle_pub = self.create_publisher(
            Twist,
            '/obstacle_avoidance',
            10
        )
        
        # State
        self.min_obstacle_distance = float('inf')
        self.obstacle_direction = 0.0
        
        self.get_logger().info('🌉 Sensor Bridge initialized')
        self.get_logger().info('   Listening for LiDAR, Depth Camera, and IMU data')
    
    def lidar_callback(self, msg: LaserScan):
        """
        Process LiDAR scan data for obstacle detection.
        
        The LaserScan message contains:
        - ranges: array of distance measurements
        - angle_min/max: scan angle range
        - angle_increment: angle between measurements
        """
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return
        
        # Find minimum distance and its direction
        min_idx = np.argmin(ranges[np.isfinite(ranges)])
        min_distance = valid_ranges.min()
        
        # Calculate angle to closest obstacle
        angle = msg.angle_min + min_idx * msg.angle_increment
        
        self.min_obstacle_distance = min_distance
        self.obstacle_direction = angle
        
        # Log if obstacle is close
        if min_distance < 1.0:
            self.get_logger().warn(
                f'⚠️ Obstacle at {min_distance:.2f}m, angle: {np.degrees(angle):.1f}°'
            )
            self._publish_avoidance_command(min_distance, angle)
    
    def depth_callback(self, msg: Image):
        """
        Process depth camera image.
        
        Depth images provide per-pixel distance measurements,
        useful for 3D perception and obstacle detection.
        """
        # Get image dimensions
        height = msg.height
        width = msg.width
        
        # Center region for forward obstacle detection
        center_x = width // 2
        center_y = height // 2
        
        # In a real implementation, you would:
        # 1. Convert to numpy array based on encoding (16UC1, 32FC1)
        # 2. Extract depth values
        # 3. Detect obstacles in the field of view
        
        # Log periodically
        if not hasattr(self, '_depth_count'):
            self._depth_count = 0
        self._depth_count += 1
        
        if self._depth_count % 30 == 0:  # Every ~1 second at 30fps
            self.get_logger().info(
                f'📷 Depth image: {width}x{height}, encoding: {msg.encoding}'
            )
    
    def imu_callback(self, msg: Imu):
        """
        Process IMU (Inertial Measurement Unit) data.
        
        IMU provides:
        - orientation: quaternion (x, y, z, w)
        - angular_velocity: rotation rates (rad/s)
        - linear_acceleration: accelerations (m/s²)
        """
        # Extract orientation (quaternion to euler would be needed for angles)
        quat = msg.orientation
        
        # Extract angular velocity
        angular = msg.angular_velocity
        
        # Extract linear acceleration
        linear = msg.linear_acceleration
        
        # Detect if robot is falling or tilting
        # Gravity should be ~9.8 m/s² on Z axis when level
        if abs(linear.z - 9.8) > 2.0:
            self.get_logger().warn(
                f'⚠️ Abnormal acceleration detected: z={linear.z:.2f} m/s²'
            )
        
        # Log periodically
        if not hasattr(self, '_imu_count'):
            self._imu_count = 0
        self._imu_count += 1
        
        if self._imu_count % 100 == 0:  # Every ~1 second at 100Hz
            self.get_logger().info(
                f'🧭 IMU: accel=({linear.x:.2f}, {linear.y:.2f}, {linear.z:.2f})'
            )
    
    def _publish_avoidance_command(self, distance: float, angle: float):
        """
        Publish obstacle avoidance velocity command.
        
        Simple reactive behavior:
        - If obstacle ahead, turn away
        - If obstacle to side, slow down
        """
        cmd = Twist()
        
        if distance < 0.5:
            # Emergency stop and turn
            cmd.linear.x = -0.1  # Back up slowly
            cmd.angular.z = 1.0 if angle > 0 else -1.0
        elif distance < 1.0:
            # Slow down and start turning
            cmd.linear.x = 0.2
            cmd.angular.z = 0.5 if angle > 0 else -0.5
        
        self.obstacle_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Shutting down Sensor Bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
