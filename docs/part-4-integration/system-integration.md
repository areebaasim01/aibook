---
sidebar_position: 1
title: "Chapter 11: System Integration Patterns"
description: Connecting all components into a cohesive robot system
keywords: [system integration, ros, robot architecture, software patterns]
---

# Chapter 11: System Integration Patterns

> *"Integration is where theory meets reality—and where most projects succeed or fail."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Design modular robot software architectures
- Implement publish-subscribe patterns for sensor data
- Create service interfaces for robot actions
- Handle system-wide error propagation

</div>

---

## 11.1 Modular Architecture

```python
from abc import ABC, abstractmethod
from typing import Dict, Any

class RobotModule(ABC):
    """Base class for robot system modules"""
    
    def __init__(self, name: str):
        self.name = name
        self.running = False
    
    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def stop(self):
        pass
    
    @abstractmethod
    def process(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        pass

class RobotSystem:
    """Main system coordinator"""
    
    def __init__(self):
        self.modules: Dict[str, RobotModule] = {}
        self.connections: Dict[str, str] = {}  # output -> input mappings
    
    def add_module(self, module: RobotModule):
        self.modules[module.name] = module
    
    def connect(self, source: str, target: str):
        """Connect output of source to input of target"""
        self.connections[source] = target
    
    def run_cycle(self, sensor_inputs: Dict):
        """Run one processing cycle through all modules"""
        data_bus = sensor_inputs.copy()
        
        for name, module in self.modules.items():
            outputs = module.process(data_bus)
            data_bus.update({f"{name}/{k}": v for k, v in outputs.items()})
        
        return data_bus
```

---

## 11.2 Pub-Sub Communication

```python
from typing import Callable, List
from collections import defaultdict

class MessageBus:
    """Central message bus for component communication"""
    
    def __init__(self):
        self.subscribers: Dict[str, List[Callable]] = defaultdict(list)
        self.last_messages: Dict[str, Any] = {}
    
    def publish(self, topic: str, message: Any):
        """Publish message to topic"""
        self.last_messages[topic] = message
        for callback in self.subscribers[topic]:
            callback(message)
    
    def subscribe(self, topic: str, callback: Callable):
        """Subscribe to topic with callback"""
        self.subscribers[topic].append(callback)
    
    def get_latest(self, topic: str) -> Any:
        """Get most recent message on topic"""
        return self.last_messages.get(topic)

# Usage example
bus = MessageBus()

# Sensor publishes data
def publish_sensor_data():
    bus.publish('/camera/image', {'data': [...]})
    bus.publish('/lidar/scan', {'points': [...]})

# Planner subscribes to sensor data
def on_camera_image(msg):
    print(f"Received image: {msg}")

bus.subscribe('/camera/image', on_camera_image)
```

---

## 11.3 Service Interface

```python
from dataclasses import dataclass
from enum import Enum

class ServiceStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    PENDING = 2

@dataclass
class ServiceRequest:
    service_name: str
    params: Dict

@dataclass
class ServiceResponse:
    status: ServiceStatus
    result: Any
    message: str = ""

class ServiceManager:
    """Manage robot services (actions that can be requested)"""
    
    def __init__(self):
        self.services: Dict[str, Callable] = {}
    
    def register(self, name: str, handler: Callable):
        self.services[name] = handler
    
    def call(self, request: ServiceRequest) -> ServiceResponse:
        """Call a service and return response"""
        if request.service_name not in self.services:
            return ServiceResponse(ServiceStatus.FAILURE, None, "Service not found")
        
        try:
            handler = self.services[request.service_name]
            result = handler(request.params)
            return ServiceResponse(ServiceStatus.SUCCESS, result)
        except Exception as e:
            return ServiceResponse(ServiceStatus.FAILURE, None, str(e))

# Register services
manager = ServiceManager()

def grasp_object(params):
    target = params['target']
    # Execute grasp...
    return {'grasped': True}

manager.register('grasp', grasp_object)
```

---

## 11.4 State Machine

```python
from enum import Enum, auto

class RobotState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    MANIPULATING = auto()
    ERROR = auto()
    SHUTDOWN = auto()

class StateMachine:
    """Robot behavioral state machine"""
    
    def __init__(self):
        self.state = RobotState.IDLE
        self.transitions: Dict[tuple, RobotState] = {}
    
    def add_transition(self, from_state: RobotState, 
                       event: str, to_state: RobotState):
        self.transitions[(from_state, event)] = to_state
    
    def trigger(self, event: str) -> bool:
        key = (self.state, event)
        if key in self.transitions:
            self.state = self.transitions[key]
            return True
        return False

# Define transitions
sm = StateMachine()
sm.add_transition(RobotState.IDLE, 'start_nav', RobotState.NAVIGATING)
sm.add_transition(RobotState.NAVIGATING, 'arrived', RobotState.IDLE)
sm.add_transition(RobotState.NAVIGATING, 'error', RobotState.ERROR)
```

---

## 📝 Exercises

1. Implement health monitoring for all modules
2. Add logging and diagnostics
3. Create a configuration system for modules

---

[← Previous: Multi-Agent Coordination](/docs/part-3-intelligence/multi-agent-coordination) | [Next: Safety & Ethics →](/docs/part-4-integration/safety-and-ethics)
