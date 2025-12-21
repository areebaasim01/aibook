---
slug: getting-started-ros2-humble
title: "Getting Started with ROS 2 Humble for Humanoid Robotics"
authors: [panaversity]
tags: [ros2, tutorial, humanoid]
---

# Your First Steps into ROS 2

**ROS 2 Humble Hawksbill** is the recommended distribution for our Physical AI course. In this post, we'll guide you through installation and your first node.

<!-- truncate -->

## Why ROS 2?

ROS 2 is to robotics what the nervous system is to the human body:

| Feature | Benefit |
|---------|---------|
| **Real-time Support** | Critical for robot control loops |
| **Security** | DDS-based secure communication |
| **Multi-platform** | Works on Linux, Windows, macOS |
| **Modern Python** | Clean `rclpy` API |

## Quick Installation

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup
source /opt/ros/humble/setup.bash

# Verify
ros2 --version
```

## Your First Node

```python
import rclpy
from rclpy.node import Node

class HelloRobot(Node):
    def __init__(self):
        super().__init__('hello_robot')
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('Hello, Robot World!')

def main():
    rclpy.init()
    node = HelloRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

Ready for more? Check out [Module 1: The Robotic Nervous System](/docs/robotic-nervous-system/) for the complete ROS 2 foundation.

---

*Learn more in the full curriculum.*
