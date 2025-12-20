---
sidebar_position: 2
title: "Chapter 2: Humanoid Robotics Fundamentals"
description: Understanding the mechanics, design principles, and control systems of humanoid robots
keywords: [humanoid robots, robot kinematics, bipedal locomotion, robot design]
---

# Chapter 2: Humanoid Robotics Fundamentals

> *"Humanoid robots are mirrors that help us understand human motion and intelligence."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Describe the anatomy and key components of humanoid robots
- Explain degrees of freedom (DoF) and their role in robot mobility
- Understand bipedal locomotion and balance challenges
- Implement basic kinematic calculations for robot limbs

</div>

---

## 2.1 Anatomy of a Humanoid Robot

Humanoid robots mimic the human form, enabling operation in human-centric environments.

### Major Subsystems

| Component | Function | Typical Sensors |
|-----------|----------|-----------------|
| **Head** | Perception & interaction | RGB-D cameras, microphones |
| **Torso** | Processing & power | IMU, temperature sensors |
| **Arms** | Manipulation | Force/torque sensors |
| **Hands** | Grasping | Tactile sensors |
| **Legs** | Locomotion | Encoders, pressure plates |

---

## 2.2 Degrees of Freedom (DoF)

**Degrees of Freedom** represent independent parameters defining a robot's configuration.

```python
from dataclasses import dataclass

@dataclass
class JointConfig:
    name: str
    dof: int
    range_deg: tuple  # (min, max)

# Typical humanoid DoF
humanoid_joints = {
    'neck': JointConfig('Neck', 2, (-45, 45)),
    'shoulder_r': JointConfig('Right Shoulder', 3, (-180, 60)),
    'elbow_r': JointConfig('Right Elbow', 1, (0, 145)),
    'hip_r': JointConfig('Right Hip', 3, (-45, 120)),
    'knee_r': JointConfig('Right Knee', 1, (0, 140)),
    'ankle_r': JointConfig('Right Ankle', 2, (-45, 30))
}

# Calculate total DoF
total = sum(j.dof for j in humanoid_joints.values()) * 2 - 2  # Arms/legs x2
print(f"Total DoF: {total}")  # ~22 DoF for basic humanoid
```

---

## 2.3 Forward and Inverse Kinematics

- **Forward Kinematics (FK)**: Joint angles → end-effector position
- **Inverse Kinematics (IK)**: End-effector position → joint angles

```python
import numpy as np

class RobotArm2D:
    """Simple 2-link planar robot arm"""
    
    def __init__(self, l1: float = 1.0, l2: float = 1.0):
        self.l1, self.l2 = l1, l2
    
    def forward_kinematics(self, theta1: float, theta2: float) -> tuple:
        """Calculate end-effector position from joint angles"""
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        return (x, y)
    
    def inverse_kinematics(self, x: float, y: float) -> list:
        """Calculate joint angles for desired position"""
        d = np.sqrt(x**2 + y**2)
        if d > self.l1 + self.l2:
            return []  # Unreachable
        
        cos_t2 = (d**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        theta2 = np.arccos(np.clip(cos_t2, -1, 1))
        
        k1 = self.l1 + self.l2 * np.cos(theta2)
        k2 = self.l2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        return [(theta1, theta2), (theta1, -theta2)]

# Usage
arm = RobotArm2D(1.0, 0.8)
x, y = arm.forward_kinematics(np.pi/4, np.pi/6)
print(f"FK: ({x:.2f}, {y:.2f})")
```

---

## 2.4 Bipedal Locomotion

Walking on two legs requires:
- **Zero Moment Point (ZMP)** control for balance
- **Gait generation** for coordinated motion
- **Impact handling** for foot strikes

```python
class WalkingGait:
    def __init__(self, step_length=0.3, step_height=0.1):
        self.step_length = step_length
        self.step_height = step_height
    
    def foot_trajectory(self, phase: float) -> dict:
        """Generate swing foot trajectory for phase [0,1]"""
        x = self.step_length * phase
        z = self.step_height * np.sin(np.pi * phase)
        return {'x': x, 'z': z}
```

---

## 2.5 Major Humanoid Platforms

| Robot | Company | DoF | Key Features |
|-------|---------|-----|--------------|
| Atlas | Boston Dynamics | 28 | Dynamic jumping |
| Optimus | Tesla | 28 | Manufacturing |
| Digit | Agility | 30 | Logistics |
| Unitree H1 | Unitree | 19 | Research |

---

## 📝 Exercises

1. Extend `RobotArm2D` to a 3-link arm
2. Visualize the walking gait using matplotlib
3. Implement ZMP calculation for balance control

---

[← Previous: Introduction to Physical AI](/docs/part-1-foundations/introduction-to-physical-ai) | [Next: AI Agent Architecture →](/docs/part-1-foundations/ai-agent-architecture)
