---
sidebar_position: 3
title: "Chapter 6: Computer Vision for Robotics"
description: Visual perception systems for robot navigation and manipulation
keywords: [computer vision, object detection, visual slam, robotics vision]
---

# Chapter 6: Computer Vision for Robotics

> *"Vision gives robots context—the difference between seeing pixels and understanding scenes."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Implement object detection for robot perception
- Understand visual SLAM concepts
- Process depth images for 3D understanding
- Apply vision to manipulation tasks

</div>

---

## 6.1 Vision Pipeline

```python
import numpy as np

class VisionPipeline:
    """Robot vision processing pipeline"""
    
    def __init__(self, detector, depth_processor):
        self.detector = detector
        self.depth = depth_processor
    
    def process(self, rgb_image: np.ndarray, 
                depth_image: np.ndarray) -> dict:
        """Process RGB-D camera frame"""
        
        # 1. Object Detection
        detections = self.detector.detect(rgb_image)
        
        # 2. Get 3D positions
        objects_3d = []
        for det in detections:
            x, y = det['center']
            z = self.depth.get_depth(depth_image, x, y)
            objects_3d.append({
                'label': det['label'],
                'confidence': det['confidence'],
                'position_3d': self.depth.pixel_to_3d(x, y, z)
            })
        
        return {'objects': objects_3d}
```

---

## 6.2 Object Detection

```python
# Using a pre-trained model (simplified)

class ObjectDetector:
    """Wrapper for object detection model"""
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.5):
        self.model = self._load_model(model_path)
        self.threshold = confidence_threshold
        self.labels = ['person', 'bottle', 'cup', 'chair', 'table']
    
    def detect(self, image: np.ndarray) -> list:
        """Run object detection on image"""
        # Forward pass through model
        outputs = self.model.infer(image)
        
        detections = []
        for box, score, class_id in zip(*outputs):
            if score > self.threshold:
                detections.append({
                    'label': self.labels[class_id],
                    'confidence': score,
                    'bbox': box,
                    'center': ((box[0]+box[2])//2, (box[1]+box[3])//2)
                })
        
        return detections
    
    def _load_model(self, path):
        # Load ONNX/TensorRT model
        return MockModel()
```

---

## 6.3 Depth Processing

```python
class DepthProcessor:
    """Process depth images to 3D coordinates"""
    
    def __init__(self, fx: float, fy: float, cx: float, cy: float):
        # Camera intrinsics
        self.fx = fx  # Focal length x
        self.fy = fy  # Focal length y
        self.cx = cx  # Principal point x
        self.cy = cy  # Principal point y
    
    def pixel_to_3d(self, u: int, v: int, depth: float) -> tuple:
        """Convert pixel + depth to 3D point"""
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return (x, y, z)
    
    def get_depth(self, depth_image: np.ndarray, u: int, v: int) -> float:
        """Get depth value at pixel with averaging"""
        # Use 3x3 region for robustness
        region = depth_image[max(0,v-1):v+2, max(0,u-1):u+2]
        valid = region[region > 0]
        return np.median(valid) if len(valid) > 0 else 0.0
```

---

## 6.4 Visual Servoing

Use vision to control robot motion:

```python
class VisualServoing:
    """Control robot to reach visual target"""
    
    def __init__(self, camera_matrix):
        self.K = camera_matrix
        self.target_center = (320, 240)  # Image center
    
    def compute_velocity(self, detection: dict) -> tuple:
        """Compute robot velocity to center target in image"""
        curr_x, curr_y = detection['center']
        
        # Error in image space
        error_x = self.target_center[0] - curr_x
        error_y = self.target_center[1] - curr_y
        
        # Simple proportional control
        gain = 0.002
        vel_x = gain * error_x  # Pan velocity
        vel_y = gain * error_y  # Tilt velocity
        
        return (vel_x, vel_y)
```

---

## 📝 Exercises

1. Implement a grasp point detector
2. Add tracking across frames
3. Create a follow-me behavior using vision

---

[← Previous: Motor Control](/docs/part-2-core-tech/motor-control) | [Next: Natural Language Interfaces →](/docs/part-2-core-tech/natural-language-interfaces)
