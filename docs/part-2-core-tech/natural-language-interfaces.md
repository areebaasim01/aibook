---
sidebar_position: 4
title: "Chapter 7: Natural Language Interfaces"
description: Enabling human-robot communication through speech and text
keywords: [nlp, speech recognition, natural language, robot interfaces]
---

# Chapter 7: Natural Language Interfaces

> *"Language transforms robots from tools into teammates."*

---

<div className="learning-objectives">

## 🎯 Learning Objectives

- Implement speech recognition for robot control
- Parse natural language commands into actions
- Design conversational robot interfaces
- Handle multi-turn dialogue with context

</div>

---

## 7.1 Speech Interface Architecture

```
User speaks → ASR → Text → NLU → Intent → Robot Action
                                ↓
                        Response Generation ← TTS ← User Hears
```

---

## 7.2 Command Parsing

```python
from dataclasses import dataclass
from typing import Optional, Dict

@dataclass
class RobotCommand:
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    quantity: Optional[int] = None

class CommandParser:
    """Parse natural language to robot commands"""
    
    def __init__(self):
        self.actions = {
            'pick': ['pick up', 'grab', 'get', 'take'],
            'place': ['put down', 'place', 'set'],
            'move': ['go to', 'move to', 'navigate'],
            'follow': ['follow', 'come with'],
        }
    
    def parse(self, text: str) -> Optional[RobotCommand]:
        """Parse text to structured command"""
        text = text.lower().strip()
        
        # Identify action
        action = None
        for cmd, phrases in self.actions.items():
            if any(phrase in text for phrase in phrases):
                action = cmd
                break
        
        if not action:
            return None
        
        # Extract target and location (simplified)
        words = text.split()
        target = self._extract_entity(words, ['cup', 'bottle', 'box'])
        location = self._extract_entity(words, ['table', 'kitchen', 'desk'])
        
        return RobotCommand(action=action, target=target, location=location)
    
    def _extract_entity(self, words: list, entities: list) -> Optional[str]:
        for word in words:
            if word in entities:
                return word
        return None

# Usage
parser = CommandParser()
cmd = parser.parse("Pick up the cup from the table")
print(cmd)  # RobotCommand(action='pick', target='cup', location='table')
```

---

## 7.3 LLM-Based Understanding

Use an LLM for more sophisticated parsing:

```python
import json

class LLMCommandParser:
    """Use LLM for natural language understanding"""
    
    def __init__(self, llm_client):
        self.llm = llm_client
    
    def parse(self, user_input: str) -> dict:
        prompt = f"""
Parse this robot command into structured JSON.

User: "{user_input}"

Available actions: move, pick, place, speak, wait
Return JSON with: action, target, location, parameters

Example:
Input: "Go to the kitchen and grab a bottle"
Output: {{"steps": [{{"action": "move", "location": "kitchen"}}, {{"action": "pick", "target": "bottle"}}]}}

Parse the input:
"""
        response = self.llm.generate(prompt)
        return json.loads(response)
```

---

## 7.4 Dialogue Management

```python
class DialogueManager:
    """Manage multi-turn robot conversations"""
    
    def __init__(self, parser, executor):
        self.parser = parser
        self.executor = executor
        self.context = {'location': 'home', 'holding': None}
    
    def process(self, user_input: str) -> str:
        # Parse command
        command = self.parser.parse(user_input)
        
        if not command:
            return "I didn't understand. Can you rephrase?"
        
        # Validate command
        if command.action == 'pick' and self.context['holding']:
            return f"I'm already holding {self.context['holding']}."
        
        # Execute
        success = self.executor.execute(command)
        
        # Update context
        if success and command.action == 'pick':
            self.context['holding'] = command.target
        
        return f"Done! I picked up the {command.target}."
```

---

## 📝 Exercises

1. Add speech synthesis for robot responses
2. Handle ambiguous commands with clarification
3. Implement command confirmation for safety

---

[← Previous: Computer Vision](/docs/part-2-core-tech/computer-vision) | [Next: Decision Making →](/docs/part-3-intelligence/decision-making)
