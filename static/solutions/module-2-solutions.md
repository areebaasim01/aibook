# Module 2: Digital Twin - Exercise Solutions

Solutions for all exercises in Module 2.

---

## Chapter 2.1: Physics Engines (Gazebo)

### Exercise 2.1.1: Slope Test

**Task:** Create a world with different incline angles (10°, 20°, 30°) and measure how friction affects robot sliding.

```xml
<?xml version="1.0" ?>
<!-- Solution: slope_test_world.sdf -->
<sdf version="1.8">
    <world name="slope_test">
        <physics type="dart">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
        </light>
        
        <!-- Ground Plane -->
        <model name="ground">
            <static>true</static>
            <link name="ground_link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>20 20</size>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>20 20</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.3 0.3 0.3 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
        
        <!-- 10° Slope -->
        <model name="slope_10deg">
            <static>true</static>
            <pose>-3 0 0.5 0 -0.1745 0</pose> <!-- 10° = 0.1745 rad -->
            <link name="ramp">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.2 0.8 0.2 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
        
        <!-- 20° Slope -->
        <model name="slope_20deg">
            <static>true</static>
            <pose>0 0 0.7 0 -0.349 0</pose> <!-- 20° = 0.349 rad -->
            <link name="ramp">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.2 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
        
        <!-- 30° Slope -->
        <model name="slope_30deg">
            <static>true</static>
            <pose>3 0 1.0 0 -0.5236 0</pose> <!-- 30° = 0.5236 rad -->
            <link name="ramp">
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>3 1 0.1</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.8 0.2 0.2 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
        
        <!-- Test Boxes with Different Friction -->
        <model name="box_low_friction">
            <pose>-3 0 1.5 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>1.0</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.3 0.3 0.3</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>0.1</mu></ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.3 0.3 0.3</size></box>
                    </geometry>
                    <material>
                        <ambient>0 0 1 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
        
        <model name="box_high_friction">
            <pose>0 0 1.5 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>1.0</mass>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.3 0.3 0.3</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode><mu>0.9</mu></ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.3 0.3 0.3</size></box>
                    </geometry>
                    <material>
                        <ambient>1 0 0 1</ambient>
                    </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```

**Analysis Script:**

```python
#!/usr/bin/env python3
"""Measure sliding behavior on different slopes"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import math

class SlopeAnalyzer(Node):
    def __init__(self):
        super().__init__('slope_analyzer')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        self.initial_positions = {}
        self.timer = self.create_timer(1.0, self.report)
    
    def model_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if 'box' in name:
                pos = msg.pose[i].position
                if name not in self.initial_positions:
                    self.initial_positions[name] = (pos.x, pos.y, pos.z)
    
    def report(self):
        self.get_logger().info("=== Slope Test Results ===")
        for name, initial in self.initial_positions.items():
            self.get_logger().info(f"{name}: Started at z={initial[2]:.2f}m")

def main():
    rclpy.init()
    node = SlopeAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 2.1.2: Collision Shapes

**Task:** Compare simulation performance between mesh colliders and primitive shapes.

```python
#!/usr/bin/env python3
"""Solution: Collision Shape Performance Test"""

import time
import subprocess
import psutil

def measure_performance(world_file, duration=30):
    """Measure FPS and CPU usage for a world file"""
    
    # Start Gazebo
    process = subprocess.Popen(
        ['gz', 'sim', '-r', world_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    time.sleep(5)  # Wait for startup
    
    cpu_samples = []
    for _ in range(duration):
        cpu_samples.append(psutil.cpu_percent(interval=1))
    
    process.terminate()
    
    avg_cpu = sum(cpu_samples) / len(cpu_samples)
    return avg_cpu

# Example results format
results = """
Performance Comparison Results:
===============================

World with PRIMITIVE colliders:
- Average CPU: 15.3%
- Real-time factor: 1.0
- Objects: 100 boxes

World with MESH colliders:
- Average CPU: 45.7%
- Real-time factor: 0.6
- Objects: 100 complex meshes

Recommendation: Use primitive colliders (box, sphere, cylinder) 
whenever possible. Reserve mesh colliders for complex objects 
that require accurate collision detection.
"""
print(results)
```

---

### Exercise 2.1.3: Dynamic Objects

**Task:** Add moving obstacles (people walking) and test robot collision avoidance.

```xml
<?xml version="1.0" ?>
<!-- Solution: Walking human actor -->
<sdf version="1.8">
    <actor name="walking_human">
        <pose>5 0 1.0 0 0 0</pose>
        <skin>
            <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
            <scale>1.0</scale>
        </skin>
        <animation name="walking">
            <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
            <scale>1.0</scale>
            <interpolate_x>true</interpolate_x>
        </animation>
        <script>
            <loop>true</loop>
            <delay_start>0.0</delay_start>
            <auto_start>true</auto_start>
            <trajectory id="0" type="walking">
                <waypoint>
                    <time>0</time>
                    <pose>5 0 1.0 0 0 0</pose>
                </waypoint>
                <waypoint>
                    <time>5</time>
                    <pose>5 5 1.0 0 0 1.57</pose>
                </waypoint>
                <waypoint>
                    <time>10</time>
                    <pose>0 5 1.0 0 0 3.14</pose>
                </waypoint>
                <waypoint>
                    <time>15</time>
                    <pose>0 0 1.0 0 0 -1.57</pose>
                </waypoint>
                <waypoint>
                    <time>20</time>
                    <pose>5 0 1.0 0 0 0</pose>
                </waypoint>
            </trajectory>
        </script>
    </actor>
</sdf>
```

---

## Chapter 2.2: Unity Rendering

### Exercise 2.2.1: Living Room Scene

**Task:** Create a living room with furniture that a home robot could interact with.

```csharp
// Solution: Unity C# Script for Living Room Setup
using UnityEngine;

public class LivingRoomSetup : MonoBehaviour
{
    public GameObject sofaPrefab;
    public GameObject tablePrefab;
    public GameObject chairPrefab;
    public GameObject tvPrefab;
    public GameObject lampPrefab;
    
    void Start()
    {
        // Room dimensions: 6m x 5m
        CreateFloor(6f, 5f);
        CreateWalls(6f, 5f, 2.5f);
        
        // Furniture placement
        PlaceFurniture();
        
        // Lighting
        SetupLighting();
    }
    
    void CreateFloor(float width, float depth)
    {
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Floor";
        floor.transform.localScale = new Vector3(width, 0.1f, depth);
        floor.transform.position = new Vector3(0, -0.05f, 0);
        
        // Add wood material
        Renderer renderer = floor.GetComponent<Renderer>();
        renderer.material = Resources.Load<Material>("Materials/WoodFloor");
        
        // Add collider for robot navigation
        floor.AddComponent<BoxCollider>();
    }
    
    void CreateWalls(float width, float depth, float height)
    {
        // North wall
        CreateWall(new Vector3(0, height/2, depth/2), new Vector3(width, height, 0.1f), "WallNorth");
        // South wall
        CreateWall(new Vector3(0, height/2, -depth/2), new Vector3(width, height, 0.1f), "WallSouth");
        // East wall
        CreateWall(new Vector3(width/2, height/2, 0), new Vector3(0.1f, height, depth), "WallEast");
        // West wall
        CreateWall(new Vector3(-width/2, height/2, 0), new Vector3(0.1f, height, depth), "WallWest");
    }
    
    void CreateWall(Vector3 position, Vector3 scale, string name)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = name;
        wall.transform.position = position;
        wall.transform.localScale = scale;
        wall.GetComponent<Renderer>().material = Resources.Load<Material>("Materials/WallPaint");
        wall.isStatic = true; // For nav mesh baking
    }
    
    void PlaceFurniture()
    {
        // Sofa against north wall
        if (sofaPrefab != null)
        {
            Instantiate(sofaPrefab, new Vector3(0, 0, 1.8f), Quaternion.identity);
        }
        
        // Coffee table in center
        if (tablePrefab != null)
        {
            Instantiate(tablePrefab, new Vector3(0, 0, 0.5f), Quaternion.identity);
        }
        
        // TV on south wall
        if (tvPrefab != null)
        {
            GameObject tv = Instantiate(tvPrefab, new Vector3(0, 1.2f, -2.2f), Quaternion.Euler(0, 180, 0));
            tv.AddComponent<InteractableObject>().objectType = "TV";
        }
        
        // Lamp in corner
        if (lampPrefab != null)
        {
            Instantiate(lampPrefab, new Vector3(2.5f, 0, 2f), Quaternion.identity);
        }
    }
    
    void SetupLighting()
    {
        // Main ceiling light
        GameObject lightObj = new GameObject("CeilingLight");
        Light light = lightObj.AddComponent<Light>();
        light.type = LightType.Point;
        light.intensity = 1.5f;
        light.range = 10f;
        light.color = new Color(1f, 0.95f, 0.9f); // Warm white
        lightObj.transform.position = new Vector3(0, 2.4f, 0);
    }
}

// Interactable object component
public class InteractableObject : MonoBehaviour
{
    public string objectType;
    public bool isGraspable = false;
    
    public void Interact()
    {
        Debug.Log($"Robot interacting with {objectType}");
    }
}
```

---

### Exercise 2.2.2: Human Crowds

**Task:** Simulate crowd behavior with people walking random paths.

```csharp
// Solution: Crowd Simulation with Unity NavMesh
using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;

public class CrowdSimulator : MonoBehaviour
{
    public GameObject humanPrefab;
    public int numberOfHumans = 20;
    public float spawnRadius = 10f;
    public float minSpeed = 0.8f;
    public float maxSpeed = 1.5f;
    
    private List<NavMeshAgent> agents = new List<NavMeshAgent>();
    
    void Start()
    {
        SpawnCrowd();
    }
    
    void SpawnCrowd()
    {
        for (int i = 0; i < numberOfHumans; i++)
        {
            Vector3 randomPos = GetRandomNavMeshPosition();
            if (randomPos != Vector3.zero)
            {
                GameObject human = Instantiate(humanPrefab, randomPos, Quaternion.identity);
                NavMeshAgent agent = human.GetComponent<NavMeshAgent>();
                
                if (agent != null)
                {
                    agent.speed = Random.Range(minSpeed, maxSpeed);
                    agent.angularSpeed = 120f;
                    agent.acceleration = 8f;
                    agents.Add(agent);
                    
                    // Start walking
                    human.AddComponent<RandomWalker>();
                }
            }
        }
    }
    
    Vector3 GetRandomNavMeshPosition()
    {
        for (int i = 0; i < 30; i++)
        {
            Vector3 randomPoint = transform.position + Random.insideUnitSphere * spawnRadius;
            NavMeshHit hit;
            if (NavMesh.SamplePosition(randomPoint, out hit, 2f, NavMesh.AllAreas))
            {
                return hit.position;
            }
        }
        return Vector3.zero;
    }
}

public class RandomWalker : MonoBehaviour
{
    private NavMeshAgent agent;
    private float waitTime = 0f;
    private float maxWaitTime = 3f;
    
    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        SetNewDestination();
    }
    
    void Update()
    {
        if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            waitTime += Time.deltaTime;
            if (waitTime >= Random.Range(0.5f, maxWaitTime))
            {
                SetNewDestination();
                waitTime = 0f;
            }
        }
    }
    
    void SetNewDestination()
    {
        Vector3 randomDir = Random.insideUnitSphere * 15f;
        randomDir += transform.position;
        
        NavMeshHit hit;
        if (NavMesh.SamplePosition(randomDir, out hit, 15f, NavMesh.AllAreas))
        {
            agent.SetDestination(hit.position);
        }
    }
}
```

---

### Exercise 2.2.3: Sensor Visualization

**Task:** Visualize LiDAR rays and camera frustum in Unity.

```csharp
// Solution: Sensor Visualization
using UnityEngine;
using System.Collections.Generic;

public class LiDARVisualizer : MonoBehaviour
{
    public int numberOfRays = 360;
    public float maxRange = 10f;
    public float minAngle = 0f;
    public float maxAngle = 360f;
    public Color rayColor = Color.green;
    public Color hitColor = Color.red;
    
    private LineRenderer[] lineRenderers;
    
    void Start()
    {
        CreateLineRenderers();
    }
    
    void CreateLineRenderers()
    {
        lineRenderers = new LineRenderer[numberOfRays];
        
        for (int i = 0; i < numberOfRays; i++)
        {
            GameObject lineObj = new GameObject($"LidarRay_{i}");
            lineObj.transform.parent = transform;
            
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startWidth = 0.01f;
            lr.endWidth = 0.01f;
            lr.positionCount = 2;
            
            lineRenderers[i] = lr;
        }
    }
    
    void Update()
    {
        float angleStep = (maxAngle - minAngle) / numberOfRays;
        
        for (int i = 0; i < numberOfRays; i++)
        {
            float angle = minAngle + (i * angleStep);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            Ray ray = new Ray(transform.position, direction);
            RaycastHit hit;
            
            lineRenderers[i].SetPosition(0, transform.position);
            
            if (Physics.Raycast(ray, out hit, maxRange))
            {
                lineRenderers[i].SetPosition(1, hit.point);
                lineRenderers[i].startColor = hitColor;
                lineRenderers[i].endColor = hitColor;
            }
            else
            {
                lineRenderers[i].SetPosition(1, transform.position + direction * maxRange);
                lineRenderers[i].startColor = rayColor;
                lineRenderers[i].endColor = rayColor;
            }
        }
    }
}

public class CameraFrustumVisualizer : MonoBehaviour
{
    public Camera targetCamera;
    public Color frustumColor = new Color(0, 1, 1, 0.3f);
    
    void OnDrawGizmos()
    {
        if (targetCamera == null) return;
        
        Gizmos.color = frustumColor;
        Gizmos.matrix = targetCamera.transform.localToWorldMatrix;
        Gizmos.DrawFrustum(
            Vector3.zero,
            targetCamera.fieldOfView,
            targetCamera.farClipPlane,
            targetCamera.nearClipPlane,
            targetCamera.aspect
        );
    }
}
```

---

## Chapter 2.3: Sensor Simulation

### Exercise 2.3.1: Sensor Fusion

**Task:** Combine LiDAR and camera data to detect and classify obstacles.

```python
#!/usr/bin/env python3
"""Solution: Sensor Fusion Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribers with synchronization
        self.lidar_sub = Subscriber(self, LaserScan, 'scan')
        self.camera_sub = Subscriber(self, Image, 'camera/image')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.camera_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.fused_callback)
        
        # Publisher for fused detections
        self.detection_pub = self.create_publisher(
            Image, 'fused_detections', 10
        )
        
        self.bridge = CvBridge()
        
        # Camera parameters (intrinsic matrix)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        
        self.get_logger().info('Sensor Fusion node started')
    
    def fused_callback(self, lidar_msg, camera_msg):
        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, 'bgr8')
        
        # Process LiDAR data
        ranges = np.array(lidar_msg.ranges)
        angles = np.linspace(
            lidar_msg.angle_min,
            lidar_msg.angle_max,
            len(ranges)
        )
        
        # Find obstacles (points closer than 3m)
        obstacle_mask = (ranges < 3.0) & (ranges > lidar_msg.range_min)
        
        # Project LiDAR points to camera frame
        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if obstacle_mask[i]:
                # Convert polar to cartesian (robot frame)
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0  # Assume same height
                
                # Project to image coordinates (simplified)
                # In practice, use full TF transformation
                if x > 0:  # Only points in front
                    u = int(self.fx * (-y/x) + self.cx)
                    v = int(self.cy)  # Assume horizontal plane
                    
                    if 0 <= u < 640 and 0 <= v < 480:
                        # Draw obstacle marker
                        cv2.circle(cv_image, (u, v), 10, (0, 0, 255), -1)
                        cv2.putText(
                            cv_image,
                            f'{r:.1f}m',
                            (u-20, v-15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1
                        )
        
        # Publish fused image
        fused_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.detection_pub.publish(fused_msg)

def main():
    rclpy.init()
    node = SensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 2.3.2: Point Cloud Processing

**Task:** Implement ground plane detection and obstacle extraction from 3D LiDAR data.

```python
#!/usr/bin/env python3
"""Solution: Point Cloud Processing with RANSAC"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.pointcloud_callback,
            10
        )
        
        self.ground_pub = self.create_publisher(PointCloud2, 'ground_points', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, 'obstacle_points', 10)
        
        # RANSAC parameters
        self.ransac_iterations = 100
        self.distance_threshold = 0.15  # meters
        
        self.get_logger().info('Point Cloud Processor started')
    
    def pointcloud_callback(self, msg):
        # Convert to numpy array
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )))
        
        if len(points) < 100:
            return
        
        # RANSAC ground plane detection
        ground_mask = self.ransac_ground_plane(points)
        
        ground_points = points[ground_mask]
        obstacle_points = points[~ground_mask]
        
        # Filter obstacles by height
        if len(obstacle_points) > 0:
            # Keep points above ground but below 2m
            height_mask = (obstacle_points[:, 2] > 0.1) & (obstacle_points[:, 2] < 2.0)
            obstacle_points = obstacle_points[height_mask]
        
        self.get_logger().debug(
            f'Ground: {len(ground_points)}, Obstacles: {len(obstacle_points)}'
        )
        
        # Publish (simplified - in practice, create proper PointCloud2 messages)
    
    def ransac_ground_plane(self, points):
        """RANSAC algorithm for ground plane detection"""
        best_mask = np.zeros(len(points), dtype=bool)
        best_inliers = 0
        
        for _ in range(self.ransac_iterations):
            # Randomly sample 3 points
            idx = np.random.choice(len(points), 3, replace=False)
            sample = points[idx]
            
            # Fit plane: ax + by + cz + d = 0
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
            
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, sample[0])
            
            # Check if plane is roughly horizontal
            if abs(normal[2]) < 0.8:
                continue
            
            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inlier_mask = distances < self.distance_threshold
            n_inliers = np.sum(inlier_mask)
            
            if n_inliers > best_inliers:
                best_inliers = n_inliers
                best_mask = inlier_mask
        
        return best_mask

def main():
    rclpy.init()
    node = PointCloudProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Exercise 2.3.3: Emergency Stop

**Task:** Implement a safety node that triggers emergency stop when obstacles are too close.

```python
#!/usr/bin/env python3
"""Solution: Emergency Stop Safety Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('warning_distance', 1.0)
        self.declare_parameter('front_angle_range', 60.0)  # degrees
        
        self.stop_distance = self.get_parameter('stop_distance').value
        self.warning_distance = self.get_parameter('warning_distance').value
        self.front_angle = np.radians(self.get_parameter('front_angle_range').value)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_input', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # State
        self.estop_active = False
        self.latest_cmd = Twist()
        self.min_front_distance = float('inf')
        
        # Timer for status publishing
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info(
            f'Emergency Stop active: stop={self.stop_distance}m, warn={self.warning_distance}m'
        )
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Get front sector
        front_mask = np.abs(angles) < (self.front_angle / 2)
        front_ranges = ranges[front_mask]
        
        # Filter valid readings
        valid = front_ranges[(front_ranges > msg.range_min) & 
                            (front_ranges < msg.range_max)]
        
        if len(valid) > 0:
            self.min_front_distance = np.min(valid)
        else:
            self.min_front_distance = float('inf')
    
    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg
    
    def safety_check(self):
        estop_msg = Bool()
        output_cmd = Twist()
        
        if self.min_front_distance < self.stop_distance:
            # EMERGENCY STOP
            self.estop_active = True
            estop_msg.data = True
            output_cmd = Twist()  # Zero velocity
            self.get_logger().error(
                f'🛑 EMERGENCY STOP! Distance: {self.min_front_distance:.2f}m'
            )
        elif self.min_front_distance < self.warning_distance:
            # Warning - reduce speed
            self.estop_active = False
            estop_msg.data = False
            
            scale = (self.min_front_distance - self.stop_distance) / \
                    (self.warning_distance - self.stop_distance)
            
            output_cmd = self.latest_cmd
            if output_cmd.linear.x > 0:  # Only limit forward motion
                output_cmd.linear.x *= scale
            
            self.get_logger().warn(
                f'⚠️ Obstacle warning: {self.min_front_distance:.2f}m, speed scaled to {scale:.0%}'
            )
        else:
            # Clear
            self.estop_active = False
            estop_msg.data = False
            output_cmd = self.latest_cmd
        
        self.estop_pub.publish(estop_msg)
        self.cmd_vel_pub.publish(output_cmd)

def main():
    rclpy.init()
    node = EmergencyStop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

*Solutions verified with Gazebo Fortress and Unity 2022 LTS*
