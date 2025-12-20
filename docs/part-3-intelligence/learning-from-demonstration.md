---
sidebar_position: 2
title: "Chapter 9: Learning from Demonstration"
description: Teaching robots through examples and demonstrations
keywords: [imitation learning, learning from demonstration, robot learning, lfd]
---

# Chapter 9: Learning from Demonstration

> *"Don't program robots—teach them."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Understand learning from demonstration (LfD) paradigms
- Implement behavior cloning from expert data
- Record and process human demonstrations
- Apply inverse reinforcement learning concepts

</div>

---

## 9.1 LfD Overview

| Method | Data Required | Pros | Cons |
|--------|---------------|------|------|
| **Behavior Cloning** | State-action pairs | Simple | Distribution shift |
| **Inverse RL** | Expert trajectories | Learns rewards | Computationally heavy |
| **GAIL** | Expert trajectories | Scalable | Requires RL |

---

## 9.2 Behavior Cloning

```python
import numpy as np
from typing import List, Tuple

class BehaviorCloning:
    """Learn policy from demonstrations using supervised learning"""
    
    def __init__(self, state_dim: int, action_dim: int):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.weights = np.random.randn(action_dim, state_dim) * 0.01
    
    def train(self, demos: List[Tuple[np.ndarray, np.ndarray]], epochs: int = 100):
        """Train on demonstration data"""
        states = np.array([d[0] for d in demos])
        actions = np.array([d[1] for d in demos])
        
        learning_rate = 0.01
        
        for epoch in range(epochs):
            # Forward pass
            predictions = states @ self.weights.T
            
            # Loss
            loss = np.mean((predictions - actions) ** 2)
            
            # Backward pass
            grad = 2 * (predictions - actions).T @ states / len(demos)
            self.weights -= learning_rate * grad
            
            if epoch % 20 == 0:
                print(f"Epoch {epoch}: Loss = {loss:.4f}")
    
    def predict(self, state: np.ndarray) -> np.ndarray:
        """Predict action for given state"""
        return state @ self.weights.T

# Example: Learn to follow a path
demos = [
    (np.array([0.0, 0.0]), np.array([1.0, 0.0])),  # At origin, move right
    (np.array([1.0, 0.0]), np.array([0.0, 1.0])),  # At (1,0), move up
    (np.array([1.0, 1.0]), np.array([-1.0, 0.0])), # At (1,1), move left
]

bc = BehaviorCloning(state_dim=2, action_dim=2)
bc.train(demos, epochs=100)
```

---

## 9.3 Recording Demonstrations

```python
from dataclasses import dataclass, field
from typing import List
import time

@dataclass
class Trajectory:
    states: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)

class DemonstrationRecorder:
    """Record human demonstrations"""
    
    def __init__(self, robot_interface):
        self.robot = robot_interface
        self.current_trajectory = None
    
    def start_recording(self):
        """Begin recording a new demonstration"""
        self.current_trajectory = Trajectory()
        print("📹 Recording started...")
    
    def record_step(self, human_action: np.ndarray):
        """Record current state and human action"""
        if self.current_trajectory is None:
            return
        
        state = self.robot.get_state()
        self.current_trajectory.states.append(state)
        self.current_trajectory.actions.append(human_action)
        self.current_trajectory.timestamps.append(time.time())
    
    def stop_recording(self) -> Trajectory:
        """End recording and return trajectory"""
        traj = self.current_trajectory
        self.current_trajectory = None
        print(f"📹 Recording stopped. {len(traj.states)} steps recorded.")
        return traj
    
    def get_training_data(self, trajectory: Trajectory) -> List[Tuple]:
        """Convert trajectory to training pairs"""
        return list(zip(trajectory.states, trajectory.actions))
```

---

## 9.4 Dynamic Movement Primitives

```python
class DMP:
    """Dynamic Movement Primitive for smooth trajectory learning"""
    
    def __init__(self, n_basis: int = 10, dt: float = 0.01):
        self.n_basis = n_basis
        self.dt = dt
        self.weights = None
    
    def learn(self, demo_trajectory: np.ndarray):
        """Learn weights from demonstration"""
        # Simplified: fit basis functions to trajectory
        T = len(demo_trajectory)
        phases = np.linspace(0, 1, T)
        
        # Gaussian basis functions
        centers = np.linspace(0, 1, self.n_basis)
        widths = 0.1
        
        Phi = np.exp(-((phases[:, None] - centers) ** 2) / widths)
        
        # Fit with least squares
        self.weights = np.linalg.lstsq(Phi, demo_trajectory, rcond=None)[0]
    
    def generate(self, num_steps: int) -> np.ndarray:
        """Generate trajectory from learned primitive"""
        phases = np.linspace(0, 1, num_steps)
        centers = np.linspace(0, 1, self.n_basis)
        
        Phi = np.exp(-((phases[:, None] - centers) ** 2) / 0.1)
        return Phi @ self.weights
```

---

## 📝 Exercises

1. Implement DAgger for iterative improvement
2. Add temporal subsampling to demonstrations
3. Learn multi-modal policies with mixture models

---

[← Previous: Decision Making](/docs/part-3-intelligence/decision-making) | [Next: Multi-Agent Coordination →](/docs/part-3-intelligence/multi-agent-coordination)
