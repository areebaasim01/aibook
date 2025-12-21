#!/usr/bin/env python3
"""
Visual SLAM Node - Simultaneous Localization and Mapping
Module 3: The AI-Robot Brain

This node implements a simple visual odometry system
that tracks camera motion and builds a map.

For production use, consider:
- Isaac ROS Visual SLAM
- ORB-SLAM3
- RTAB-Map

Usage:
    ros2 run my_robot_pkg slam_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple


@dataclass
class Landmark:
    """A 3D landmark in the map"""
    id: int
    position: np.ndarray  # (x, y, z)
    descriptor: np.ndarray
    observations: int = 1


class VisualSLAMNode(Node):
    """
    Visual SLAM node for robot localization and mapping.
    
    This is a simplified implementation demonstrating:
    - Feature detection and tracking
    - Pose estimation
    - Map construction
    - TF transform broadcasting
    """
    
    def __init__(self):
        super().__init__('visual_slam')
        
        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # QoS for camera
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            camera_qos
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/slam/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/slam/map', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # SLAM state
        self.camera_matrix: Optional[np.ndarray] = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.landmarks: List[Landmark] = []
        self.prev_features = None
        self.prev_image = None
        self.frame_count = 0
        
        # Map
        self.map_resolution = 0.05  # meters per pixel
        self.map_size = 200  # pixels
        self.occupancy_map = np.ones((self.map_size, self.map_size)) * 50  # Unknown
        
        # Timer for publishing map
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('🗺️ Visual SLAM node initialized')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'📷 Camera matrix received: fx={msg.k[0]:.1f}')
    
    def image_callback(self, msg: Image):
        """Process incoming camera images for SLAM"""
        self.frame_count += 1
        
        if self.camera_matrix is None:
            return
        
        # Convert ROS image to numpy (simplified - assumes mono8 or rgb8)
        height = msg.height
        width = msg.width
        
        # In production, use cv_bridge for proper conversion
        # Here we simulate feature detection
        
        # Simulate feature detection (in production, use ORB/SIFT)
        num_features = np.random.randint(100, 200)
        current_features = self._detect_features(width, height, num_features)
        
        if self.prev_features is not None:
            # Match features with previous frame
            matches = self._match_features(self.prev_features, current_features)
            
            if len(matches) > 10:
                # Estimate motion from matched features
                delta_pose = self._estimate_motion(matches)
                
                # Update current pose
                self.current_pose = self.current_pose @ delta_pose
                
                # Update map with new observations
                self._update_map()
        
        # Store for next frame
        self.prev_features = current_features
        
        # Publish pose and odometry
        self._publish_pose()
        self._broadcast_transform()
        
        if self.frame_count % 30 == 0:
            pos = self.current_pose[:3, 3]
            self.get_logger().info(
                f'📍 Pose: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})'
            )
    
    def _detect_features(
        self, 
        width: int, 
        height: int, 
        num_features: int
    ) -> np.ndarray:
        """
        Simulate feature detection.
        
        In production, use:
        - cv2.ORB_create()
        - cv2.SIFT_create()
        - cv2.goodFeaturesToTrack()
        """
        features = np.random.rand(num_features, 2)
        features[:, 0] *= width
        features[:, 1] *= height
        return features
    
    def _match_features(
        self,
        prev_features: np.ndarray,
        curr_features: np.ndarray
    ) -> List[Tuple[int, int]]:
        """
        Simulate feature matching.
        
        In production, use:
        - cv2.BFMatcher
        - cv2.FlannBasedMatcher
        """
        matches = []
        num_matches = min(len(prev_features), len(curr_features), 50)
        
        for i in range(num_matches):
            matches.append((i, i))
        
        return matches
    
    def _estimate_motion(
        self,
        matches: List[Tuple[int, int]]
    ) -> np.ndarray:
        """
        Estimate camera motion from feature matches.
        
        In production, use:
        - cv2.recoverPose() with Essential matrix
        - cv2.solvePnP() with 3D-2D correspondences
        """
        # Simulate small random motion
        delta = np.eye(4)
        
        # Small translation
        delta[0, 3] = np.random.randn() * 0.01  # x
        delta[1, 3] = np.random.randn() * 0.01  # y
        delta[2, 3] = 0.02  # z (forward motion)
        
        # Small rotation around z-axis
        angle = np.random.randn() * 0.01
        c, s = np.cos(angle), np.sin(angle)
        delta[:2, :2] = np.array([[c, -s], [s, c]])
        
        return delta
    
    def _update_map(self):
        """Update occupancy map with new observations"""
        # Get current position in map coordinates
        pos = self.current_pose[:3, 3]
        map_x = int(pos[0] / self.map_resolution + self.map_size / 2)
        map_y = int(pos[1] / self.map_resolution + self.map_size / 2)
        
        # Mark current position as free
        if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
            # Decrease occupancy probability (more free)
            self.occupancy_map[map_y, map_x] = max(
                0, 
                self.occupancy_map[map_y, map_x] - 5
            )
    
    def _publish_pose(self):
        """Publish current pose estimate"""
        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        
        pose_msg.pose.position.x = self.current_pose[0, 3]
        pose_msg.pose.position.y = self.current_pose[1, 3]
        pose_msg.pose.position.z = self.current_pose[2, 3]
        
        # Convert rotation matrix to quaternion (simplified)
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        
        self.odom_pub.publish(odom_msg)
    
    def _broadcast_transform(self):
        """Broadcast TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        t.transform.translation.x = self.current_pose[0, 3]
        t.transform.translation.y = self.current_pose[1, 3]
        t.transform.translation.z = self.current_pose[2, 3]
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_map(self):
        """Publish occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_size
        map_msg.info.height = self.map_size
        map_msg.info.origin.position.x = -self.map_size * self.map_resolution / 2
        map_msg.info.origin.position.y = -self.map_size * self.map_resolution / 2
        
        # Convert to int8 (-1 unknown, 0-100 occupancy)
        map_data = self.occupancy_map.flatten().astype(np.int8).tolist()
        map_msg.data = map_data
        
        self.map_pub.publish(map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualSLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Shutting down Visual SLAM')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
