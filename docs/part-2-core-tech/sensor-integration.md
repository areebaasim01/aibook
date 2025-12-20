---
sidebar_position: 1
title: "Chapter 4: Sensor Integration & Perception"
description: Building the sensory systems that enable robots to perceive the world
keywords: [sensors, perception, lidar, camera, imu, sensor fusion]
---

# Chapter 4: Sensor Integration & Perception

> *"Sensors are the eyes, ears, and skin of a robot—without them, intelligence is blind."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Understand common sensor types used in robotics
- Implement sensor data processing pipelines
- Apply sensor fusion techniques for robust perception
- Handle noise and uncertainty in sensor readings

</div>

---

## 4.1 Common Robot Sensors

| Sensor Type | Function | Output | Range |
|-------------|----------|--------|-------|
| **RGB Camera** | Color images | Pixels | 0.5-∞m |
| **Depth Camera** | 3D distances | Point cloud | 0.3-10m |
| **LiDAR** | Laser ranging | Point cloud | 0.1-100m |
| **IMU** | Acceleration/rotation | 6-axis data | N/A |
| **Encoders** | Joint positions | Angles | N/A |
| **Force/Torque** | Contact forces | 6-axis forces | N/A |

---

## 4.2 Sensor Data Processing

```python
import numpy as np
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class SensorReading:
    value: float
    timestamp: float
    valid: bool = True

class NoiseFilter:
    """Filter noisy sensor data"""
    
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.buffer: List[float] = []
    
    def update(self, reading: SensorReading) -> float:
        """Add new reading and return filtered value"""
        if not reading.valid:
            return self.get_current()
        
        self.buffer.append(reading.value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        return np.median(self.buffer)
    
    def get_current(self) -> float:
        return np.median(self.buffer) if self.buffer else 0.0

# Example: Filter noisy distance readings
distance_filter = NoiseFilter(window_size=5)
raw_readings = [1.52, 1.48, 1.55, 1.47, 5.0, 1.51]  # Includes outlier

for r in raw_readings:
    filtered = distance_filter.update(SensorReading(r, 0))
    print(f"Raw: {r:.2f} → Filtered: {filtered:.2f}")
```

---

## 4.3 Sensor Fusion

Combining multiple sensors for more accurate perception:

```python
class SensorFusion:
    """Combine multiple sensors using weighted average"""
    
    def __init__(self, weights: List[float]):
        self.weights = np.array(weights)
        self.weights /= self.weights.sum()  # Normalize
    
    def fuse(self, readings: List[Optional[float]]) -> float:
        """Fuse multiple sensor readings"""
        valid = [(r, w) for r, w in zip(readings, self.weights) if r is not None]
        if not valid:
            return 0.0
        
        values, weights = zip(*valid)
        weights = np.array(weights)
        weights /= weights.sum()
        
        return np.average(values, weights=weights)

# Fuse camera depth, lidar, and ultrasonic sensors
fusion = SensorFusion(weights=[1.0, 2.0, 0.5])  # Trust LiDAR more
distance = fusion.fuse([1.45, 1.52, 1.48])
print(f"Fused distance: {distance:.2f}m")
```

---

## 4.4 Camera Perception

```python
# Pseudocode for camera processing pipeline

def process_camera_frame(frame):
    # 1. Preprocessing
    frame = undistort(frame)
    frame = normalize(frame)
    
    # 2. Object Detection
    objects = detect_objects(frame)  # YOLO, SSD, etc.
    
    # 3. Depth Estimation (if RGB-D)
    depths = estimate_depth(frame)
    
    # 4. Semantic Segmentation
    segments = segment_scene(frame)
    
    return {
        'objects': objects,
        'depths': depths,
        'segments': segments
    }
```

---

## 4.5 LiDAR Processing

```python
def process_lidar_scan(points: np.ndarray) -> dict:
    """Process 2D/3D LiDAR point cloud"""
    
    # Filter points
    valid = points[:, 2] > 0.1  # Remove ground points
    points = points[valid]
    
    # Cluster obstacles
    clusters = cluster_points(points, eps=0.5)
    
    # Extract obstacle info
    obstacles = []
    for cluster in clusters:
        center = cluster.mean(axis=0)
        radius = np.max(np.linalg.norm(cluster - center, axis=1))
        obstacles.append({'center': center, 'radius': radius})
    
    return {'obstacles': obstacles, 'points': points}
```

---

## 📝 Exercises

1. Implement a Kalman filter for sensor fusion
2. Create a multi-sensor perception pipeline
3. Handle sensor dropout and failure scenarios

---

[← Previous: AI Agent Architecture](/docs/part-1-foundations/ai-agent-architecture) | [Next: Motor Control →](/docs/part-2-core-tech/motor-control)
