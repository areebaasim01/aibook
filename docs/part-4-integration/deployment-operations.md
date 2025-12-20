---
sidebar_position: 3
title: "Chapter 13: Deployment & Operations"
description: Deploying and operating physical AI systems in production
keywords: [deployment, devops, robot operations, production systems, mlops]
---

# Chapter 13: Deployment & Operations

> *"A robot in the lab is a prototype. A robot in the field is a product."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Deploy robot software to production hardware
- Implement monitoring and observability
- Handle over-the-air updates
- Design for reliability and uptime

</div>

---

## 13.1 Deployment Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    CLOUD SERVICES                        │
│  ┌─────────┐  ┌──────────┐  ┌──────────────────────┐   │
│  │ Metrics │  │ Logs     │  │ Model Registry       │   │
│  └────┬────┘  └────┬─────┘  └──────────┬───────────┘   │
├───────┼────────────┼────────────────────┼───────────────┤
│       │     SECURE GATEWAY              │               │
├───────┼────────────┼────────────────────┼───────────────┤
│       ↓            ↓                    ↓               │
│  ┌─────────────────────────────────────────────────┐   │
│  │              ROBOT FLEET                         │   │
│  │  Robot 1    Robot 2    Robot 3    ...           │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

---

## 13.2 Health Monitoring

```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List

@dataclass
class HealthStatus:
    component: str
    healthy: bool
    message: str
    timestamp: datetime = field(default_factory=datetime.now)

class HealthMonitor:
    """Monitor robot system health"""
    
    def __init__(self):
        self.checks: Dict[str, callable] = {}
        self.history: List[HealthStatus] = []
    
    def register_check(self, name: str, check_fn: callable):
        """Register a health check"""
        self.checks[name] = check_fn
    
    def run_checks(self) -> Dict[str, HealthStatus]:
        """Run all health checks"""
        results = {}
        
        for name, check_fn in self.checks.items():
            try:
                healthy, message = check_fn()
                status = HealthStatus(name, healthy, message)
            except Exception as e:
                status = HealthStatus(name, False, f"Check failed: {e}")
            
            results[name] = status
            self.history.append(status)
        
        return results
    
    def is_healthy(self) -> bool:
        """Quick overall health check"""
        results = self.run_checks()
        return all(s.healthy for s in results.values())

# Example health checks
monitor = HealthMonitor()

def check_battery():
    level = get_battery_level()
    return level > 20, f"Battery: {level}%"

def check_network():
    latency = measure_network_latency()
    return latency < 100, f"Latency: {latency}ms"

monitor.register_check('battery', check_battery)
monitor.register_check('network', check_network)
```

---

## 13.3 Over-the-Air Updates

```python
import hashlib
from pathlib import Path

class OTAUpdater:
    """Over-the-air software update system"""
    
    def __init__(self, update_url: str, current_version: str):
        self.update_url = update_url
        self.current_version = current_version
    
    def check_for_updates(self) -> dict:
        """Check if updates are available"""
        manifest = self._fetch_manifest()
        
        if manifest['version'] > self.current_version:
            return {
                'available': True,
                'version': manifest['version'],
                'size': manifest['size'],
                'changes': manifest['changelog']
            }
        return {'available': False}
    
    def download_update(self, target_path: Path) -> bool:
        """Download update package"""
        manifest = self._fetch_manifest()
        
        # Download with progress
        self._download_file(
            manifest['download_url'],
            target_path,
            manifest['checksum']
        )
        
        # Verify checksum
        return self._verify_checksum(target_path, manifest['checksum'])
    
    def apply_update(self, package_path: Path) -> bool:
        """Apply downloaded update"""
        try:
            # 1. Create backup
            self._backup_current()
            
            # 2. Stop services
            self._stop_services()
            
            # 3. Apply update
            self._extract_and_install(package_path)
            
            # 4. Restart services
            self._start_services()
            
            return True
        except Exception as e:
            # Rollback on failure
            self._rollback()
            raise e
```

---

## 13.4 Logging and Telemetry

```python
import json
import time
from queue import Queue
from threading import Thread

class RobotTelemetry:
    """Collect and transmit robot telemetry"""
    
    def __init__(self, robot_id: str, endpoint: str):
        self.robot_id = robot_id
        self.endpoint = endpoint
        self.buffer: Queue = Queue()
        self.running = False
    
    def log(self, category: str, data: dict):
        """Log telemetry data point"""
        entry = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'category': category,
            'data': data
        }
        self.buffer.put(entry)
    
    def start_transmission(self, batch_size: int = 100):
        """Start background transmission"""
        self.running = True
        
        def transmit_loop():
            batch = []
            while self.running:
                while not self.buffer.empty() and len(batch) < batch_size:
                    batch.append(self.buffer.get())
                
                if batch:
                    self._send_batch(batch)
                    batch = []
                time.sleep(1)
        
        Thread(target=transmit_loop, daemon=True).start()
    
    def _send_batch(self, batch: list):
        # Send to telemetry endpoint
        pass

# Usage
telemetry = RobotTelemetry('robot-001', 'https://api.example.com/telemetry')
telemetry.start_transmission()

# Log various metrics
telemetry.log('motion', {'velocity': 0.5, 'position': (1.2, 3.4)})
telemetry.log('battery', {'level': 85, 'voltage': 24.1})
```

---

## 13.5 Deployment Checklist

Before deploying to production:

- [ ] All safety systems tested
- [ ] Emergency stop verified
- [ ] Network connectivity stable
- [ ] Backup power tested
- [ ] Software version locked
- [ ] Monitoring alerts configured
- [ ] Rollback procedure documented
- [ ] User training completed

---

## 📝 Exercises

1. Implement A/B testing for robot behaviors
2. Create a dashboard for fleet monitoring
3. Design a staged rollout system

---

## 🎉 Congratulations!

You've completed the AI-Native Textbook on Physical AI & Humanoid Robotics. You now have the knowledge to:

- Build intelligent physical AI systems
- Design humanoid robot software
- Deploy safe, ethical robot systems
- Operate robot fleets at scale

Continue learning by building projects and joining the community!

---

[← Previous: Safety & Ethics](/docs/part-4-integration/safety-and-ethics) | [Back to Introduction →](/docs/intro)
