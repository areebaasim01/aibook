---
sidebar_position: 1
title: "Chapter 8: Decision Making & Planning"
description: Algorithms for robot planning and decision making under uncertainty
keywords: [path planning, decision making, motion planning, robot planning]
---

# Chapter 8: Decision Making & Planning

> *"A robot without planning is reactive; with planning, it becomes purposeful."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Implement path planning algorithms (A*, RRT)
- Design decision-making systems for autonomous behavior
- Handle uncertainty in planning
- Create hierarchical task planners

</div>

---

## 8.1 Path Planning with A*

```python
import heapq
from typing import List, Tuple, Optional

class GridPlanner:
    """A* path planner for 2D grid"""
    
    def __init__(self, grid_size: Tuple[int, int]):
        self.width, self.height = grid_size
        self.obstacles = set()
    
    def add_obstacle(self, x: int, y: int):
        self.obstacles.add((x, y))
    
    def plan(self, start: Tuple[int, int], 
             goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Find shortest path using A*"""
        
        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])
        
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                neighbor = (current[0]+dx, current[1]+dy)
                
                if not self._valid(neighbor):
                    continue
                
                new_g = g_score[current] + 1
                
                if neighbor not in g_score or new_g < g_score[neighbor]:
                    g_score[neighbor] = new_g
                    f_score = new_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current
        
        return None  # No path found
    
    def _valid(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                pos not in self.obstacles)

# Usage
planner = GridPlanner((10, 10))
planner.add_obstacle(3, 4)
planner.add_obstacle(3, 5)
path = planner.plan((0, 0), (9, 9))
print(f"Path: {path}")
```

---

## 8.2 Behavior Trees

Structured decision making for complex behaviors:

```python
from enum import Enum
from abc import ABC, abstractmethod

class Status(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BTNode(ABC):
    @abstractmethod
    def tick(self) -> Status:
        pass

class Sequence(BTNode):
    """Run children until one fails"""
    def __init__(self, children: List[BTNode]):
        self.children = children
    
    def tick(self) -> Status:
        for child in self.children:
            status = child.tick()
            if status != Status.SUCCESS:
                return status
        return Status.SUCCESS

class Selector(BTNode):
    """Run children until one succeeds"""
    def __init__(self, children: List[BTNode]):
        self.children = children
    
    def tick(self) -> Status:
        for child in self.children:
            status = child.tick()
            if status == Status.SUCCESS:
                return Status.SUCCESS
        return Status.FAILURE

class Action(BTNode):
    """Leaf node that performs an action"""
    def __init__(self, name: str, action_fn):
        self.name = name
        self.action_fn = action_fn
    
    def tick(self) -> Status:
        return self.action_fn()

# Example: Pick and place behavior tree
def check_object(): return Status.SUCCESS
def move_to_object(): return Status.SUCCESS
def grasp(): return Status.SUCCESS
def move_to_goal(): return Status.SUCCESS
def release(): return Status.SUCCESS

pick_place_bt = Sequence([
    Action("Check Object", check_object),
    Action("Move to Object", move_to_object),
    Action("Grasp", grasp),
    Action("Move to Goal", move_to_goal),
    Action("Release", release)
])
```

---

## 8.3 Task Planning

```python
from dataclasses import dataclass
from typing import Set, Dict

@dataclass
class State:
    predicates: Set[str]

class TaskPlanner:
    """Simple STRIPS-like task planner"""
    
    def __init__(self, actions: Dict):
        self.actions = actions
    
    def plan(self, initial: State, goal: Set[str]) -> List[str]:
        """Find plan to achieve goal"""
        if goal.issubset(initial.predicates):
            return []
        
        for action_name, action in self.actions.items():
            pre, add, delete = action
            
            if pre.issubset(initial.predicates):
                new_state = State(
                    (initial.predicates - delete) | add
                )
                sub_plan = self.plan(new_state, goal)
                if sub_plan is not None:
                    return [action_name] + sub_plan
        
        return None

# Define actions: (preconditions, add effects, delete effects)
actions = {
    'pick_cup': ({'at_table', 'gripper_empty'}, {'holding_cup'}, {'gripper_empty'}),
    'place_cup': ({'at_counter', 'holding_cup'}, {'cup_on_counter', 'gripper_empty'}, {'holding_cup'}),
    'move_to_table': (set(), {'at_table'}, {'at_counter'}),
    'move_to_counter': (set(), {'at_counter'}, {'at_table'}),
}
```

---

## 📝 Exercises

1. Implement RRT for continuous spaces
2. Add conditions and decorators to behavior trees
3. Create a hierarchical planner with sub-tasks

---

[← Previous: Natural Language](/docs/part-2-core-tech/natural-language-interfaces) | [Next: Learning from Demonstration →](/docs/part-3-intelligence/learning-from-demonstration)
