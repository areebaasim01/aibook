---
sidebar_position: 3
title: "Chapter 3: AI Agent Architecture"
description: Designing intelligent agents that bridge software and physical systems
keywords: [ai agents, agent architecture, llm agents, embodied ai, robot control]
---

# Chapter 3: AI Agent Architecture

> *"An AI agent is the bridge between intelligence and action—the mind that drives the machine."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Understand the components of an AI agent architecture
- Design agents that interface with physical systems
- Implement perception-reasoning-action pipelines
- Integrate Large Language Models (LLMs) as robot controllers

</div>

---

## 3.1 What is an AI Agent?

An **AI Agent** is an autonomous system that:
- Perceives its environment
- Reasons about goals and constraints
- Takes actions to achieve objectives
- Learns from feedback

### Agent Architecture Layers

```
┌────────────────────────────────────────────┐
│           COGNITIVE LAYER                  │
│  Planning │ Reasoning │ Learning           │
├────────────────────────────────────────────┤
│           INTERFACE LAYER                  │
│  Perception │ Action │ Communication       │
├────────────────────────────────────────────┤
│           PHYSICAL LAYER                   │
│  Sensors │ Actuators │ Hardware            │
└────────────────────────────────────────────┘
```

---

## 3.2 Agent Components

```python
from abc import ABC, abstractmethod
from typing import Dict, Any, List
from dataclasses import dataclass

@dataclass
class Observation:
    """Raw sensor data from environment"""
    sensors: Dict[str, Any]
    timestamp: float

@dataclass
class Action:
    """Command to execute"""
    name: str
    params: Dict[str, Any]

class PhysicalAIAgent(ABC):
    """Base class for embodied AI agents"""
    
    def __init__(self, name: str):
        self.name = name
        self.memory: List[Dict] = []
    
    @abstractmethod
    def perceive(self, observation: Observation) -> Dict:
        """Convert raw sensors to semantic understanding"""
        pass
    
    @abstractmethod
    def reason(self, state: Dict, goal: str) -> Action:
        """Decide what action to take"""
        pass
    
    @abstractmethod
    def act(self, action: Action) -> bool:
        """Execute the chosen action"""
        pass
    
    def step(self, observation: Observation, goal: str) -> bool:
        """One agent step: perceive → reason → act"""
        state = self.perceive(observation)
        action = self.reason(state, goal)
        return self.act(action)
```

---

## 3.3 LLM-Powered Agents

Modern agents leverage Large Language Models for reasoning:

```python
from typing import Optional
import json

class LLMRobotAgent(PhysicalAIAgent):
    """Robot agent powered by an LLM for reasoning"""
    
    def __init__(self, name: str, llm_client):
        super().__init__(name)
        self.llm = llm_client
        self.available_actions = [
            "move_forward", "turn_left", "turn_right",
            "pick_up", "put_down", "speak"
        ]
    
    def perceive(self, observation: Observation) -> Dict:
        """Convert observations to text description"""
        return {
            'objects': observation.sensors.get('detected_objects', []),
            'distance_ahead': observation.sensors.get('lidar_front', 999),
            'position': observation.sensors.get('position', (0, 0))
        }
    
    def reason(self, state: Dict, goal: str) -> Action:
        """Use LLM to decide next action"""
        prompt = f"""
You are a robot controller. Given the current state and goal,
choose the best action.

State: {json.dumps(state)}
Goal: {goal}
Available actions: {self.available_actions}

Respond with JSON: {{"action": "action_name", "params": {{}}}}
"""
        response = self.llm.generate(prompt)
        result = json.loads(response)
        return Action(name=result['action'], params=result.get('params', {}))
    
    def act(self, action: Action) -> bool:
        """Execute action on robot hardware"""
        print(f"🤖 Executing: {action.name}({action.params})")
        return True
```

---

## 3.4 ReAct Pattern for Robots

The **ReAct** (Reasoning + Acting) pattern enables step-by-step problem solving:

```python
class ReActRobotAgent:
    """Agent using ReAct pattern for complex tasks"""
    
    def __init__(self, llm_client):
        self.llm = llm_client
        self.trace = []
    
    def solve(self, task: str, max_steps: int = 5) -> str:
        """Solve task using iterative reasoning and acting"""
        context = f"Task: {task}\n"
        
        for step in range(max_steps):
            # Think
            thought = self._think(context)
            context += f"Thought {step+1}: {thought}\n"
            
            # Act
            action = self._decide_action(thought)
            context += f"Action {step+1}: {action}\n"
            
            # Observe
            result = self._execute(action)
            context += f"Observation {step+1}: {result}\n"
            
            # Check completion
            if "DONE" in result:
                return context
        
        return context
    
    def _think(self, context: str) -> str:
        return self.llm.generate(f"{context}\nThink about what to do next:")
    
    def _decide_action(self, thought: str) -> str:
        return self.llm.generate(f"Based on: {thought}\nWhat action?")
    
    def _execute(self, action: str) -> str:
        # Simulate action execution
        return f"Executed {action}"
```

---

## 3.5 Agent Communication

Robots often work together. Here's a multi-agent pattern:

```python
from queue import Queue
from threading import Thread

class AgentMessage:
    def __init__(self, sender: str, receiver: str, content: Dict):
        self.sender = sender
        self.receiver = receiver
        self.content = content

class MultiAgentSystem:
    """Coordinate multiple robot agents"""
    
    def __init__(self):
        self.agents: Dict[str, PhysicalAIAgent] = {}
        self.message_queues: Dict[str, Queue] = {}
    
    def register(self, agent: PhysicalAIAgent):
        self.agents[agent.name] = agent
        self.message_queues[agent.name] = Queue()
    
    def send_message(self, msg: AgentMessage):
        if msg.receiver in self.message_queues:
            self.message_queues[msg.receiver].put(msg)
    
    def broadcast(self, sender: str, content: Dict):
        for name in self.agents:
            if name != sender:
                self.send_message(AgentMessage(sender, name, content))
```

---

## 3.6 Summary

| Component | Purpose |
|-----------|---------|
| **Perception** | Convert sensors to semantic state |
| **Reasoning** | Decide actions based on goals |
| **Action** | Execute commands on hardware |
| **Memory** | Store past experiences |
| **Communication** | Coordinate with other agents |

---

## 📝 Exercises

1. Implement a goal-directed agent with multiple objectives
2. Add memory/context to the LLM agent for multi-turn tasks
3. Create a master-worker multi-agent system

---

[← Previous: Humanoid Robotics](/docs/part-1-foundations/humanoid-robotics-fundamentals) | [Next: Sensor Integration →](/docs/part-2-core-tech/sensor-integration)
