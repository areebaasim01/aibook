---
sidebar_label: "1.2 Python Bridging (rclpy)"
sidebar_position: 3
title: "Python Bridging: Connecting AI Agents with Hardware"
description: "Implementation of rclpy to bridge AI agents with robot hardware"
keywords: [rclpy, python, ros2, ai agents, hardware interface]
---

# 1.2 Python Bridging with rclpy

> *"rclpy is the bridge between Python's AI ecosystem and ROS 2's real-time robotics world."*

---

##  Learning Objectives

- Set up a ROS 2 Python package from scratch
- Implement publishers, subscribers, and services in Python
- Create the "Hello Robot" node deliverable
- Bridge AI agent logic with ROS 2 communication

---

## Package Structure

A well-organized ROS 2 Python package:

```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── hello_robot_node.py
│   ├── sensor_bridge.py
│   └── ai_agent_interface.py
├── resource/
│   └── my_robot_pkg
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Creating Your First Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_robot_pkg

# Build
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# Source
source install/setup.bash
```

---

##  Deliverable: Hello Robot Node

Here's the complete "Hello Robot" node that serves as your first ROS 2 application:

```python
#!/usr/bin/env python3
"""
Hello Robot Node - Your first ROS 2 application
This node demonstrates core ROS 2 concepts with rclpy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class HelloRobotNode(Node):
    """
    A comprehensive introductory ROS 2 node demonstrating:
    - Publishing to multiple topics
    - Subscribing to commands
    - Timer-based callbacks
    - Logging and diagnostics
    """
    
    def __init__(self):
        super().__init__('hello_robot')
        
        # Node parameters
        self.declare_parameter('robot_name', 'PAI-Humanoid')
        self.declare_parameter('publish_rate', 10.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, 
            'robot_status', 
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Timers
        self.status_timer = self.create_timer(
            1.0 / self.rate,
            self.publish_status
        )
        
        self.joint_timer = self.create_timer(
            0.02,  # 50Hz for joint states
            self.publish_joints
        )
        
        # State
        self.startup_time = time.time()
        self.joint_positions = [0.0] * 6  # 6 DOF arm
        self.current_velocity = Twist()
        
        self.get_logger().info(f'🤖 {self.robot_name} initialized!')
        self.get_logger().info(f'   Publishing status at {self.rate}Hz')
    
    def publish_status(self):
        """Publish robot status message"""
        uptime = time.time() - self.startup_time
        
        msg = String()
        msg.data = (
            f'{self.robot_name} | '
            f'Uptime: {uptime:.1f}s | '
            f'Status: OPERATIONAL'
        )
        
        self.status_pub.publish(msg)
    
    def publish_joints(self):
        """Publish simulated joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.name = [
            'shoulder_pan', 'shoulder_lift', 'elbow',
            'wrist_1', 'wrist_2', 'wrist_3'
        ]
        msg.position = self.joint_positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        
        self.joint_pub.publish(msg)
    
    def cmd_callback(self, msg: Twist):
        """Handle velocity commands"""
        self.current_velocity = msg
        self.get_logger().info(
            f'Received cmd_vel: linear={msg.linear.x:.2f}, '
            f'angular={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = HelloRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Setup Configuration

**setup.py:**

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Physical AI Robot Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hello_robot = my_robot_pkg.hello_robot_node:main',
        ],
    },
)
```

---

## AI Agent Interface

Bridge your AI agents with ROS 2:

```python
"""
AI Agent Interface - Connect LLM agents to robot actions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json


class AIAgentBridge(Node):
    """
    Bridge between AI agents (LLMs) and ROS 2 robot systems
    """
    
    def __init__(self):
        super().__init__('ai_agent_bridge')
        
        # Command input from AI agent
        self.command_sub = self.create_subscription(
            String,
            'ai_agent/commands',
            self.process_agent_command,
            10
        )
        
        # Feedback to AI agent
        self.feedback_pub = self.create_publisher(
            String,
            'ai_agent/feedback',
            10
        )
        
        # Robot state for context
        self.robot_state = {
            'position': [0, 0, 0],
            'battery': 100,
            'status': 'idle'
        }
        
        self.get_logger().info('AI Agent Bridge initialized')
    
    def process_agent_command(self, msg: String):
        """
        Process commands from AI agent
        Expected format: JSON with 'action' and 'params'
        """
        try:
            command = json.loads(msg.data)
            action = command.get('action')
            params = command.get('params', {})
            
            self.get_logger().info(f'AI Command: {action}')
            
            # Route to appropriate handler
            if action == 'move_to':
                self.handle_move_to(params)
            elif action == 'pick_object':
                self.handle_pick(params)
            elif action == 'speak':
                self.handle_speak(params)
            else:
                self.send_feedback('error', f'Unknown action: {action}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid command JSON: {e}')
            self.send_feedback('error', 'Invalid command format')
    
    def handle_move_to(self, params: dict):
        """Handle navigation command"""
        target = params.get('target')
        self.get_logger().info(f'Moving to: {target}')
        # Publish to navigation stack
        self.send_feedback('executing', f'Navigating to {target}')
    
    def handle_pick(self, params: dict):
        """Handle manipulation command"""
        obj = params.get('object')
        self.get_logger().info(f'Picking: {obj}')
        self.send_feedback('executing', f'Attempting to pick {obj}')
    
    def handle_speak(self, params: dict):
        """Handle speech output"""
        text = params.get('text', '')
        self.get_logger().info(f'Speaking: {text}')
        self.send_feedback('completed', f'Spoke: {text}')
    
    def send_feedback(self, status: str, message: str):
        """Send feedback to AI agent"""
        feedback = String()
        feedback.data = json.dumps({
            'status': status,
            'message': message,
            'robot_state': self.robot_state
        })
        self.feedback_pub.publish(feedback)
```

---

## Running the Node

```bash
# Build and source
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run with default parameters
ros2 run my_robot_pkg hello_robot

# Run with custom parameters
ros2 run my_robot_pkg hello_robot --ros-args \
    -p robot_name:="Atlas-v2" \
    -p publish_rate:=20.0

# In another terminal, echo the status
ros2 topic echo /robot_status
```

:::tip Using Launch Files
For complex systems, use launch files instead of running nodes individually. See the next section for launch file examples.
:::

---

## Exercises

### Exercise 1.2.1: Sensor Publisher
Extend the HelloRobotNode to publish simulated IMU data on the `/imu/data` topic.

### Exercise 1.2.2: Command Interpreter
Create a node that subscribes to text commands and converts them to `Twist` messages.

### Exercise 1.2.3: AI Agent Integration
Build a complete AI agent interface that accepts natural language and outputs ROS 2 actions.

---

<div style={{textAlign: 'center', marginTop: '2rem'}}>

[← Previous: ROS 2 Fundamentals](./ros2-fundamentals.md) | [Next: Humanoid URDF →](./humanoid-urdf.md)

</div>
