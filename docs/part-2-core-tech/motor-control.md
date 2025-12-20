---
sidebar_position: 2
title: "Chapter 5: Motor Control & Actuation"
description: Understanding motor systems and implementing control algorithms
keywords: [motor control, pid, actuators, servo, robotics control]
---

# Chapter 5: Motor Control & Actuation

> *"Motors are the muscles of robotics—without precise control, even the smartest AI is paralyzed."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Understand different actuator types and their characteristics
- Implement PID control for position and velocity
- Design trajectory generators for smooth motion
- Handle motor safety and fault tolerance

</div>

---

## 5.1 Actuator Types

| Type | Use Case | Pros | Cons |
|------|----------|------|------|
| **DC Motor** | Wheels, joints | Simple, cheap | Requires encoder |
| **Servo** | Precise positioning | Built-in feedback | Limited rotation |
| **Stepper** | 3D printers | Open-loop control | Can skip steps |
| **Brushless** | Drones, fast motion | High efficiency | Complex driver |

---

## 5.2 PID Control

The **Proportional-Integral-Derivative** controller is fundamental:

```python
class PIDController:
    """Classic PID controller for motor control"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = (-1.0, 1.0)
    
    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """Calculate control output"""
        error = setpoint - measurement
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        # Sum and clamp
        output = p_term + i_term + d_term
        return max(self.output_limits[0], min(output, self.output_limits[1]))
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# Position control example
pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
target = 90.0  # degrees
current = 0.0

for _ in range(50):
    control = pid.update(target, current, dt=0.01)
    current += control * 5  # Simulated motor response
    print(f"Target: {target:.1f}, Current: {current:.1f}")
```

---

## 5.3 Trajectory Generation

Smooth motion requires planned trajectories:

```python
import numpy as np

def trapezoidal_profile(start: float, end: float, 
                       max_vel: float, max_acc: float,
                       dt: float) -> np.ndarray:
    """Generate trapezoidal velocity profile"""
    distance = abs(end - start)
    direction = 1 if end > start else -1
    
    # Calculate times
    t_acc = max_vel / max_acc
    d_acc = 0.5 * max_acc * t_acc ** 2
    
    if 2 * d_acc > distance:
        # Triangle profile (can't reach max velocity)
        t_acc = np.sqrt(distance / max_acc)
        t_cruise = 0
    else:
        t_cruise = (distance - 2 * d_acc) / max_vel
    
    t_total = 2 * t_acc + t_cruise
    
    # Generate trajectory
    positions = []
    t = 0
    while t <= t_total:
        if t < t_acc:
            pos = 0.5 * max_acc * t ** 2
        elif t < t_acc + t_cruise:
            pos = d_acc + max_vel * (t - t_acc)
        else:
            t_dec = t - t_acc - t_cruise
            pos = distance - 0.5 * max_acc * (t_acc - t_dec) ** 2
        
        positions.append(start + direction * pos)
        t += dt
    
    return np.array(positions)

# Generate smooth motion from 0 to 90 degrees
trajectory = trapezoidal_profile(0, 90, max_vel=45, max_acc=90, dt=0.01)
```

---

## 5.4 Safety Systems

```python
class SafeMotorController:
    """Motor controller with safety features"""
    
    def __init__(self, controller: PIDController):
        self.pid = controller
        self.position_limits = (-180, 180)
        self.velocity_limit = 100  # deg/s
        self.current_limit = 2.0  # Amps
        self.enabled = False
    
    def command(self, target: float, current_pos: float, 
                current_vel: float, current_amps: float, dt: float) -> float:
        
        if not self.enabled:
            return 0.0
        
        # Check position limits
        if current_pos < self.position_limits[0] or \
           current_pos > self.position_limits[1]:
            self.enabled = False
            return 0.0
        
        # Check overcurrent
        if current_amps > self.current_limit:
            return 0.0
        
        # Run PID
        return self.pid.update(target, current_pos, dt)
```

---

## 📝 Exercises

1. Tune PID gains for a simulated motor
2. Implement a quintic polynomial trajectory
3. Add anti-windup to the PID controller

---

[← Previous: Sensor Integration](/docs/part-2-core-tech/sensor-integration) | [Next: Computer Vision →](/docs/part-2-core-tech/computer-vision)
