# Module 1: Robotic Nervous System - Exercise Solutions

Solutions for all exercises in Module 1.

---

## Chapter 1.1: ROS 2 Fundamentals

### Exercise 1.1.1: Topic Communication

**Task:** Create two nodes - a temperature_publisher that publishes simulated temperature readings, and a temperature_monitor that subscribes and logs warnings above 50°C.

```python
#!/usr/bin/env python3
"""Solution: Temperature Publisher Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info('Temperature Publisher started')
    
    def publish_temperature(self):
        msg = Float32()
        # Simulate temperature between 20-80°C with occasional spikes
        msg.data = random.uniform(20.0, 80.0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published temperature: {msg.data:.1f}°C')

def main():
    rclpy.init()
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
#!/usr/bin/env python3
"""Solution: Temperature Monitor Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        self.threshold = 50.0
        self.get_logger().info(f'Monitoring temperature (warning threshold: {self.threshold}°C)')
    
    def temperature_callback(self, msg):
        temp = msg.data
        if temp > self.threshold:
            self.get_logger().warn(f'⚠️ HIGH TEMPERATURE: {temp:.1f}°C (above {self.threshold}°C)')
        else:
            self.get_logger().info(f'Temperature: {temp:.1f}°C - OK')

def main():
    rclpy.init()
    node = TemperatureMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 1.1.2: Service Implementation

**Task:** Create a service that accepts a robot joint name and returns its current angle.

```python
#!/usr/bin/env python3
"""Solution: Joint Angle Service Server"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from std_srvs.srv import SetBool
import random

# Custom service definition would go in a .srv file
# For this example, we'll use a simplified approach

class JointAngleService(Node):
    def __init__(self):
        super().__init__('joint_angle_service')
        
        # Simulated joint states
        self.joint_angles = {
            'hip_left': 0.0,
            'hip_right': 0.0,
            'knee_left': 0.15,
            'knee_right': 0.15,
            'ankle_left': -0.1,
            'ankle_right': -0.1,
        }
        
        # Create service (using Trigger as simplified example)
        self.srv = self.create_service(
            Trigger,
            'get_joint_angles',
            self.get_angles_callback
        )
        
        # Simulate joint movement
        self.timer = self.create_timer(0.1, self.update_joints)
        self.get_logger().info('Joint Angle Service ready')
    
    def update_joints(self):
        """Simulate small joint movements"""
        for joint in self.joint_angles:
            self.joint_angles[joint] += random.uniform(-0.01, 0.01)
    
    def get_angles_callback(self, request, response):
        """Return all joint angles as formatted string"""
        angles_str = "\n".join([
            f"  {name}: {angle:.3f} rad" 
            for name, angle in self.joint_angles.items()
        ])
        response.success = True
        response.message = f"Joint Angles:\n{angles_str}"
        return response

def main():
    rclpy.init()
    node = JointAngleService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 1.1.3: Graph Analysis

**Task:** Run `rqt_graph` with the `turtlesim` package and identify all nodes and topics.

**Solution:**

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Start teleop
ros2 run turtlesim turtle_teleop_key

# Terminal 3: Visualize the graph
ros2 run rqt_graph rqt_graph
```

**Expected Graph Structure:**

```
Nodes:
- /turtlesim
- /teleop_turtle

Topics:
- /turtle1/cmd_vel (geometry_msgs/msg/Twist)
- /turtle1/pose (turtlesim/msg/Pose)
- /turtle1/color_sensor (turtlesim/msg/Color)

Services (on /turtlesim):
- /clear
- /kill
- /spawn
- /turtle1/set_pen
- /turtle1/teleport_absolute
- /turtle1/teleport_relative
```

---

## Chapter 1.2: Python Bridging

### Exercise 1.2.1: Sensor Publisher

**Task:** Create a node that reads from a simulated IMU (accelerometer + gyroscope) and publishes sensor_msgs/Imu messages.

```python
#!/usr/bin/env python3
"""Solution: IMU Publisher Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import math
import random

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz
        
        # Simulation state
        self.time = 0.0
        self.get_logger().info('IMU Publisher started (50 Hz)')
    
    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Simulated orientation (quaternion)
        roll = 0.1 * math.sin(self.time)
        pitch = 0.05 * math.sin(self.time * 0.5)
        yaw = 0.02 * self.time
        msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
        
        # Simulated angular velocity
        msg.angular_velocity = Vector3(
            x=0.1 * math.cos(self.time) + random.gauss(0, 0.01),
            y=0.05 * math.cos(self.time * 0.5) + random.gauss(0, 0.01),
            z=0.02 + random.gauss(0, 0.005)
        )
        
        # Simulated linear acceleration (gravity + motion)
        msg.linear_acceleration = Vector3(
            x=random.gauss(0, 0.1),
            y=random.gauss(0, 0.1),
            z=9.81 + random.gauss(0, 0.1)  # Gravity
        )
        
        # Covariance (simplified)
        msg.orientation_covariance = [0.01] * 9
        msg.angular_velocity_covariance = [0.001] * 9
        msg.linear_acceleration_covariance = [0.01] * 9
        
        self.publisher.publish(msg)
        self.time += 0.02
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy
        )

def main():
    rclpy.init()
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 1.2.2: Command Interpreter

**Task:** Build a node that subscribes to String commands ("forward", "left", "stop") and publishes corresponding Twist messages.

```python
#!/usr/bin/env python3
"""Solution: Command Interpreter Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')
        
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.command_callback,
            10
        )
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Command mappings
        self.commands = {
            'forward': {'linear': 0.5, 'angular': 0.0},
            'backward': {'linear': -0.5, 'angular': 0.0},
            'left': {'linear': 0.0, 'angular': 0.5},
            'right': {'linear': 0.0, 'angular': -0.5},
            'stop': {'linear': 0.0, 'angular': 0.0},
            'fast_forward': {'linear': 1.0, 'angular': 0.0},
            'turn_around': {'linear': 0.0, 'angular': 3.14},
        }
        
        self.get_logger().info('Command Interpreter ready')
        self.get_logger().info(f'Available commands: {list(self.commands.keys())}')
    
    def command_callback(self, msg):
        command = msg.data.lower().strip()
        
        if command in self.commands:
            twist = Twist()
            twist.linear.x = self.commands[command]['linear']
            twist.angular.z = self.commands[command]['angular']
            
            self.publisher.publish(twist)
            self.get_logger().info(
                f'Command "{command}" → linear: {twist.linear.x}, angular: {twist.angular.z}'
            )
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')

def main():
    rclpy.init()
    node = CommandInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 1.2.3: AI Agent Integration

**Task:** Connect a simple AI model (e.g., a decision tree) that receives sensor data and outputs movement commands.

```python
#!/usr/bin/env python3
"""Solution: Simple AI Agent Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # AI parameters (simple rule-based for demo)
        self.min_distance = 0.5  # meters
        self.forward_speed = 0.3
        self.turn_speed = 0.5
        
        self.get_logger().info('AI Agent initialized (obstacle avoidance)')
    
    def scan_callback(self, msg):
        """Process LiDAR data and decide movement"""
        ranges = np.array(msg.ranges)
        
        # Handle infinite values
        ranges = np.where(np.isinf(ranges), 10.0, ranges)
        ranges = np.where(np.isnan(ranges), 10.0, ranges)
        
        # Divide scan into regions
        n_ranges = len(ranges)
        left = ranges[0:n_ranges//3]
        front = ranges[n_ranges//3:2*n_ranges//3]
        right = ranges[2*n_ranges//3:]
        
        # Get minimum distances in each region
        left_min = np.min(left) if len(left) > 0 else 10.0
        front_min = np.min(front) if len(front) > 0 else 10.0
        right_min = np.min(right) if len(right) > 0 else 10.0
        
        # Decision tree
        twist = Twist()
        
        if front_min > self.min_distance:
            # Clear ahead - go forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            decision = "FORWARD"
        elif left_min > right_min:
            # Turn left
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            decision = "TURN LEFT"
        else:
            # Turn right
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed
            decision = "TURN RIGHT"
        
        self.publisher.publish(twist)
        self.get_logger().debug(
            f'{decision} (front: {front_min:.2f}, left: {left_min:.2f}, right: {right_min:.2f})'
        )

def main():
    rclpy.init()
    node = SimpleAIAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Chapter 1.3: URDF

### Exercise 1.3.1: Add Arms

**Task:** Extend the bipedal URDF with arms (shoulder, elbow, wrist joints).

```xml
<!-- Solution: Arm URDF extension (add to bipedal_robot.urdf) -->

<!-- Left Shoulder -->
<link name="left_shoulder">
    <visual>
        <geometry>
            <sphere radius="0.04"/>
        </geometry>
        <material name="joint_color"/>
    </visual>
    <collision>
        <geometry>
            <sphere radius="0.04"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.3"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
</link>

<joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.2 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="3.14" effort="50" velocity="2.0"/>
</joint>

<!-- Left Upper Arm -->
<link name="left_upper_arm">
    <visual>
        <origin xyz="-0.15 0 0" rpy="0 1.57 0"/>
        <geometry>
            <cylinder radius="0.03" length="0.3"/>
        </geometry>
        <material name="arm_color">
            <color rgba="0.8 0.6 0.4 1"/>
        </material>
    </visual>
    <collision>
        <origin xyz="-0.15 0 0" rpy="0 1.57 0"/>
        <geometry>
            <cylinder radius="0.03" length="0.3"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
</link>

<joint name="left_upper_arm_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="40" velocity="2.0"/>
</joint>

<!-- Left Elbow -->
<link name="left_elbow">
    <visual>
        <geometry>
            <sphere radius="0.025"/>
        </geometry>
        <material name="joint_color"/>
    </visual>
</link>

<joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_elbow"/>
    <origin xyz="-0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="30" velocity="2.5"/>
</joint>

<!-- Left Forearm -->
<link name="left_forearm">
    <visual>
        <origin xyz="-0.125 0 0" rpy="0 1.57 0"/>
        <geometry>
            <cylinder radius="0.025" length="0.25"/>
        </geometry>
        <material name="arm_color"/>
    </visual>
    <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
</link>

<joint name="left_forearm_joint" type="fixed">
    <parent link="left_elbow"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Left Wrist -->
<link name="left_hand">
    <visual>
        <geometry>
            <box size="0.1 0.06 0.02"/>
        </geometry>
        <material name="hand_color">
            <color rgba="0.9 0.7 0.5 1"/>
        </material>
    </visual>
    <inertial>
        <mass value="0.3"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
</link>

<joint name="left_wrist_joint" type="revolute">
    <parent link="left_forearm"/>
    <child link="left_hand"/>
    <origin xyz="-0.25 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="3.0"/>
</joint>

<!-- Mirror for right arm (change y values to negative) -->
<!-- ... similar structure with "right_" prefix ... -->
```

---

### Exercise 1.3.2: Add Sensors

**Task:** Add camera and LiDAR sensor frames to the head of your humanoid.

```xml
<!-- Solution: Sensor URDF extension -->

<!-- Camera Mount -->
<link name="camera_link">
    <visual>
        <geometry>
            <box size="0.03 0.08 0.02"/>
        </geometry>
        <material name="camera_color">
            <color rgba="0.1 0.1 0.1 1"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <box size="0.03 0.08 0.02"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
</link>

<joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.08 0 0.02" rpy="0 0 0"/>
</joint>

<!-- Camera Optical Frame (rotated for ROS conventions) -->
<link name="camera_optical_frame"/>

<joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>

<!-- LiDAR Mount -->
<link name="lidar_link">
    <visual>
        <geometry>
            <cylinder radius="0.04" length="0.05"/>
        </geometry>
        <material name="lidar_color">
            <color rgba="0.3 0.3 0.3 1"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="0.04" length="0.05"/>
        </geometry>
    </collision>
    <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
</link>

<joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
</joint>

<!-- IMU Sensor -->
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo Sensor Plugins -->
<gazebo reference="camera_link">
    <sensor type="camera" name="head_camera">
        <update_rate>30.0</update_rate>
        <camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <ros>
                <namespace>/robot</namespace>
                <remapping>image_raw:=camera/image</remapping>
                <remapping>camera_info:=camera/info</remapping>
            </ros>
            <frame_name>camera_optical_frame</frame_name>
        </plugin>
    </sensor>
</gazebo>

<gazebo reference="lidar_link">
    <sensor type="ray" name="head_lidar">
        <update_rate>10</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>
                <max>10.0</max>
            </range>
        </ray>
        <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
            <ros>
                <namespace>/robot</namespace>
                <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>lidar_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

---

### Exercise 1.3.3: Use Xacro

**Task:** Convert your URDF to Xacro with macros for repeated elements (legs, arms).

See the complete solution file: `module-1/humanoid_xacro.xacro`

---

*Solutions verified with ROS 2 Humble on Ubuntu 22.04*
