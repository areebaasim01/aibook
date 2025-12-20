---
sidebar_position: 3
title: "Chapter 10: Multi-Agent Coordination"
description: Coordinating multiple robots and agents for complex tasks
keywords: [multi-robot, coordination, swarm robotics, multi-agent systems]
---

# Chapter 10: Multi-Agent Coordination

> *"One robot is powerful. A coordinated team is unstoppable."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Design multi-robot communication protocols
- Implement task allocation algorithms
- Coordinate formation control
- Handle distributed decision making

</div>

---

## 10.1 Communication Architecture

```python
from dataclasses import dataclass
from typing import Dict, List, Callable
from queue import Queue

@dataclass
class Message:
    sender_id: str
    msg_type: str
    content: dict
    timestamp: float

class RobotNetwork:
    """Communication network for multi-robot system"""
    
    def __init__(self):
        self.robots: Dict[str, Queue] = {}
        self.message_handlers: Dict[str, Callable] = {}
    
    def register(self, robot_id: str, handler: Callable):
        """Register a robot in the network"""
        self.robots[robot_id] = Queue()
        self.message_handlers[robot_id] = handler
    
    def send(self, msg: Message, target_id: str):
        """Send message to specific robot"""
        if target_id in self.robots:
            self.robots[target_id].put(msg)
    
    def broadcast(self, msg: Message):
        """Send message to all robots"""
        for robot_id, queue in self.robots.items():
            if robot_id != msg.sender_id:
                queue.put(msg)
    
    def process_messages(self, robot_id: str):
        """Process pending messages for a robot"""
        queue = self.robots[robot_id]
        while not queue.empty():
            msg = queue.get()
            self.message_handlers[robot_id](msg)
```

---

## 10.2 Task Allocation

```python
from typing import List, Set
import numpy as np

class TaskAllocator:
    """Allocate tasks to robots based on capabilities and cost"""
    
    def __init__(self, robot_ids: List[str]):
        self.robots = robot_ids
        self.assigned: Dict[str, str] = {}  # task -> robot
    
    def allocate(self, tasks: List[dict], robot_positions: Dict[str, tuple]) -> Dict:
        """Greedy task allocation based on distance"""
        available_robots = set(self.robots)
        allocation = {}
        
        for task in sorted(tasks, key=lambda t: t.get('priority', 0), reverse=True):
            best_robot = None
            best_cost = float('inf')
            
            for robot_id in available_robots:
                cost = self._compute_cost(
                    robot_positions[robot_id],
                    task['location']
                )
                if cost < best_cost:
                    best_cost = cost
                    best_robot = robot_id
            
            if best_robot:
                allocation[task['id']] = best_robot
                available_robots.remove(best_robot)
        
        return allocation
    
    def _compute_cost(self, robot_pos: tuple, task_pos: tuple) -> float:
        return np.sqrt(sum((a-b)**2 for a, b in zip(robot_pos, task_pos)))

# Usage
allocator = TaskAllocator(['robot_1', 'robot_2', 'robot_3'])
tasks = [
    {'id': 'pickup_A', 'location': (1, 2), 'priority': 1},
    {'id': 'pickup_B', 'location': (5, 3), 'priority': 2},
]
positions = {'robot_1': (0, 0), 'robot_2': (4, 4), 'robot_3': (2, 1)}
assignment = allocator.allocate(tasks, positions)
```

---

## 10.3 Formation Control

```python
class FormationController:
    """Maintain robot formation while moving"""
    
    def __init__(self, formation_offsets: Dict[str, tuple]):
        # Offsets relative to formation center
        self.offsets = formation_offsets
    
    def compute_velocities(self, 
                          positions: Dict[str, np.ndarray],
                          formation_center: np.ndarray,
                          velocity_goal: np.ndarray) -> Dict[str, np.ndarray]:
        """Compute velocities to maintain formation"""
        velocities = {}
        
        for robot_id, offset in self.offsets.items():
            # Desired position
            goal_pos = formation_center + np.array(offset)
            current_pos = positions[robot_id]
            
            # Position error
            error = goal_pos - current_pos
            
            # Proportional control + formation velocity
            kp = 2.0
            velocities[robot_id] = kp * error + velocity_goal
        
        return velocities

# Triangle formation
formation = FormationController({
    'leader': (0.0, 0.0),
    'follower_1': (-1.0, -1.0),
    'follower_2': (1.0, -1.0),
})
```

---

## 10.4 Consensus Algorithms

```python
class ConsensusProtocol:
    """Distributed consensus for multi-robot agreement"""
    
    def __init__(self, robot_ids: List[str], adjacency: Dict[str, List[str]]):
        self.robots = robot_ids
        self.neighbors = adjacency  # Who can communicate with whom
    
    def reach_consensus(self, 
                        initial_values: Dict[str, float],
                        iterations: int = 10) -> Dict[str, float]:
        """Average consensus algorithm"""
        values = initial_values.copy()
        
        for _ in range(iterations):
            new_values = {}
            
            for robot_id in self.robots:
                # Average with neighbors
                neighbor_values = [values[n] for n in self.neighbors[robot_id]]
                avg = (values[robot_id] + sum(neighbor_values)) / (1 + len(neighbor_values))
                new_values[robot_id] = avg
            
            values = new_values
        
        return values
```

---

## 📝 Exercises

1. Implement market-based task allocation
2. Add collision avoidance to formation control
3. Create a leader-follower coordination system

---

[← Previous: Learning from Demonstration](/docs/part-3-intelligence/learning-from-demonstration) | [Next: System Integration →](/docs/part-4-integration/system-integration)
