#!/usr/bin/env python3
"""
VLA Orchestrator - Complete Vision-Language-Action System
Module 4: Vision-Language-Action

This is the capstone component that integrates:
- Speech recognition (Whisper)
- Language understanding (LLM)
- Vision processing
- Action execution (Nav2, manipulation)
- Speech output (TTS)

Usage:
    ros2 run my_robot_pkg vla_orchestrator
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import json
import asyncio
from enum import Enum
from typing import List, Dict, Any, Optional
from dataclasses import dataclass


class RobotState(Enum):
    """Current robot state"""
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    EXECUTING = "executing"
    SPEAKING = "speaking"
    ERROR = "error"


@dataclass 
class Location:
    """Named location with coordinates"""
    name: str
    x: float
    y: float
    yaw: float = 0.0


class VLAOrchestrator(Node):
    """
    Vision-Language-Action Orchestrator
    
    The main controller that:
    1. Receives voice commands
    2. Uses LLM to plan actions
    3. Executes actions via ROS 2
    4. Provides voice feedback
    """
    
    # Predefined locations (in a real system, from a map)
    LOCATIONS = {
        "home_base": Location("home_base", 0.0, 0.0, 0.0),
        "kitchen": Location("kitchen", 5.0, 2.0, 0.0),
        "living_room": Location("living_room", 0.0, 0.0, 0.0),
        "bedroom": Location("bedroom", -3.0, 4.0, 1.57),
        "bathroom": Location("bathroom", -2.0, -2.0, 3.14),
        "office": Location("office", 2.0, -3.0, -1.57),
    }
    
    def __init__(self):
        super().__init__('vla_orchestrator')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State
        self.state = RobotState.IDLE
        self.current_location = "home_base"
        self.holding = None
        self.action_queue: List[Dict] = []
        
        # ---------- SUBSCRIBERS ----------
        
        # Voice commands from Whisper node
        self.command_sub = self.create_subscription(
            String,
            'speech/command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Camera for vision
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ---------- PUBLISHERS ----------
        
        # Speech output
        self.speech_pub = self.create_publisher(String, 'speech/output', 10)
        
        # Status updates
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        
        # ---------- ACTION CLIENTS ----------
        
        # Navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # ---------- TIMERS ----------
        
        # Main loop
        self.main_timer = self.create_timer(
            0.1,
            self.main_loop,
            callback_group=self.callback_group
        )
        
        # Status publisher
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('🤖 VLA Orchestrator initialized')
        self.speak("Hello! I am ready for voice commands.")
    
    # ========== CALLBACKS ==========
    
    def command_callback(self, msg: String):
        """Handle incoming voice commands"""
        command = msg.data
        self.get_logger().info(f'📢 Received command: "{command}"')
        
        if self.state != RobotState.IDLE:
            self.speak("I'm busy right now. Please wait.")
            return
        
        # Queue the command for processing
        self.state = RobotState.THINKING
        self.speak(f"Processing: {command}")
        
        # In production, call LLM here
        actions = self._mock_plan_actions(command)
        self.action_queue = actions
        
        self.state = RobotState.IDLE
    
    def camera_callback(self, msg: Image):
        """Handle camera images for object detection"""
        # In production, run object detection here
        pass
    
    # ========== ACTION PLANNING ==========
    
    def _mock_plan_actions(self, command: str) -> List[Dict]:
        """
        Mock action planning (replace with LLM in production).
        
        Simple keyword-based parsing for testing.
        """
        actions = []
        command_lower = command.lower()
        
        # Check for navigation commands
        for loc_name in self.LOCATIONS.keys():
            if loc_name.replace("_", " ") in command_lower:
                actions.append({
                    "action": "navigate",
                    "location": loc_name,
                    "description": f"Going to {loc_name}"
                })
                break
        
        # Check for pick commands
        if "pick" in command_lower or "get" in command_lower or "grab" in command_lower:
            # Extract object (simplified)
            words = command_lower.split()
            for i, word in enumerate(words):
                if word in ["pick", "get", "grab"] and i + 1 < len(words):
                    obj = words[i + 1]
                    if obj in ["up", "the", "a"]:
                        if i + 2 < len(words):
                            obj = words[i + 2]
                    actions.append({
                        "action": "pick",
                        "object": obj,
                        "description": f"Picking up {obj}"
                    })
                    break
        
        # Check for speak commands
        if "say" in command_lower or "tell" in command_lower:
            actions.append({
                "action": "speak",
                "text": "Hello, I am your robot assistant!",
                "description": "Speaking greeting"
            })
        
        # Default: acknowledge
        if not actions:
            actions.append({
                "action": "speak",
                "text": f"I heard: {command}. Let me help you with that.",
                "description": "Acknowledging command"
            })
        
        return actions
    
    # ========== ACTION EXECUTION ==========
    
    def main_loop(self):
        """Main execution loop"""
        if self.state != RobotState.IDLE or not self.action_queue:
            return
        
        # Get next action
        action = self.action_queue.pop(0)
        self.execute_action(action)
    
    def execute_action(self, action: Dict):
        """Execute a single action"""
        action_type = action.get("action")
        self.state = RobotState.EXECUTING
        
        self.get_logger().info(f'⚙️ Executing: {action.get("description")}')
        
        if action_type == "navigate":
            self.execute_navigate(action)
        elif action_type == "pick":
            self.execute_pick(action)
        elif action_type == "place":
            self.execute_place(action)
        elif action_type == "speak":
            self.execute_speak(action)
        elif action_type == "look":
            self.execute_look(action)
        else:
            self.get_logger().warn(f'Unknown action: {action_type}')
            self.state = RobotState.IDLE
    
    def execute_navigate(self, action: Dict):
        """Execute navigation action"""
        location_name = action.get("location")
        
        if location_name not in self.LOCATIONS:
            self.speak(f"I don't know where {location_name} is.")
            self.state = RobotState.IDLE
            return
        
        location = self.LOCATIONS[location_name]
        self.speak(f"Navigating to {location_name}")
        
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = location.x
        goal.pose.pose.position.y = location.y
        
        # Orientation from yaw
        import math
        goal.pose.pose.orientation.z = math.sin(location.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(location.yaw / 2)
        
        # Send goal
        if self.nav_client.wait_for_server(timeout_sec=5.0):
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                lambda f: self._navigation_complete(f, location_name)
            )
        else:
            self.speak("Navigation system not available")
            self.state = RobotState.IDLE
    
    def _navigation_complete(self, future, location_name: str):
        """Callback when navigation completes"""
        self.current_location = location_name
        self.speak(f"I have arrived at {location_name}")
        self.state = RobotState.IDLE
    
    def execute_pick(self, action: Dict):
        """Execute pick action"""
        obj = action.get("object")
        self.speak(f"Looking for {obj}")
        
        # Simulate picking (in production, use manipulation actions)
        import time
        time.sleep(1.0)
        
        self.holding = obj
        self.speak(f"I have picked up the {obj}")
        self.state = RobotState.IDLE
    
    def execute_place(self, action: Dict):
        """Execute place action"""
        surface = action.get("surface")
        
        if not self.holding:
            self.speak("I'm not holding anything")
            self.state = RobotState.IDLE
            return
        
        obj = self.holding
        self.speak(f"Placing {obj} on {surface}")
        
        # Simulate placing
        import time
        time.sleep(1.0)
        
        self.holding = None
        self.speak(f"Done! {obj} is on the {surface}")
        self.state = RobotState.IDLE
    
    def execute_speak(self, action: Dict):
        """Execute speak action"""
        text = action.get("text", "Hello!")
        self.speak(text)
        self.state = RobotState.IDLE
    
    def execute_look(self, action: Dict):
        """Execute look/search action"""
        obj = action.get("object")
        self.speak(f"Looking for {obj}")
        
        # In production, use vision to find object
        import time
        time.sleep(1.0)
        
        self.speak(f"I found the {obj}")
        self.state = RobotState.IDLE
    
    # ========== HELPERS ==========
    
    def speak(self, text: str):
        """Publish text for TTS"""
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f'🔊 {text}')
    
    def publish_status(self):
        """Publish current status"""
        status = {
            "state": self.state.value,
            "location": self.current_location,
            "holding": self.holding,
            "queue_length": len(self.action_queue)
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLAOrchestrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.speak("Goodbye!")
        node.get_logger().info('👋 Shutting down VLA Orchestrator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
