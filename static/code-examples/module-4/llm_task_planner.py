#!/usr/bin/env python3
"""
LLM Task Planner - Natural Language to Robot Actions
Module 4: Vision-Language-Action

This module uses OpenAI GPT-4 or Anthropic Claude to convert
natural language commands into structured robot action sequences.

Usage:
    from llm_task_planner import TaskPlanner
    
    planner = TaskPlanner()
    actions = await planner.plan("Go to the kitchen and pick up the red cup")
"""

import os
import json
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from enum import Enum
from openai import AsyncOpenAI
# from anthropic import AsyncAnthropic  # Alternative


class ActionType(Enum):
    """Available robot actions"""
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    LOOK = "look"
    SPEAK = "speak"
    WAIT = "wait"
    WAVE = "wave"
    FOLLOW = "follow"


@dataclass
class RobotAction:
    """A single robot action"""
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str
    
    def to_dict(self) -> dict:
        return {
            "action": self.action_type.value,
            "params": self.parameters,
            "description": self.description
        }


@dataclass
class RobotContext:
    """Current robot state for context-aware planning"""
    location: str = "home_base"
    holding: Optional[str] = None
    battery_level: int = 100
    known_locations: List[str] = None
    nearby_objects: List[str] = None
    
    def __post_init__(self):
        if self.known_locations is None:
            self.known_locations = [
                "home_base", "kitchen", "living_room", 
                "bedroom", "bathroom", "office"
            ]
        if self.nearby_objects is None:
            self.nearby_objects = []
    
    def to_dict(self) -> dict:
        return {
            "current_location": self.location,
            "holding": self.holding,
            "battery_level": self.battery_level,
            "known_locations": self.known_locations,
            "nearby_objects": self.nearby_objects
        }


class TaskPlanner:
    """
    LLM-based task planner for converting natural language to robot actions.
    
    Key features:
    - Context-aware planning
    - Multi-step action sequences
    - Safety validation
    - Structured output parsing
    """
    
    SYSTEM_PROMPT = """You are a robot task planner. Your job is to convert
natural language commands into structured action sequences.

AVAILABLE ACTIONS:
1. navigate: Move to a location
   Parameters: {"location": "kitchen" | "living_room" | "bedroom" | "bathroom" | "office" | "home_base"}
   
2. pick: Pick up an object
   Parameters: {"object": "cup" | "book" | "remote" | etc, "color": "optional"}
   
3. place: Place held object somewhere
   Parameters: {"surface": "table" | "counter" | "shelf" | etc}
   
4. look: Search for an object
   Parameters: {"object": "what to find"}
   
5. speak: Say something to the user
   Parameters: {"text": "message to speak"}
   
6. wait: Wait for a duration
   Parameters: {"seconds": 5}
   
7. wave: Wave at someone
   Parameters: {"direction": "forward" | "left" | "right"}
   
8. follow: Follow a person
   Parameters: {"target": "person description"}

SAFETY RULES:
- Never pick up dangerous objects (knives, scissors, hot items)
- Always confirm before approaching unknown people
- Check battery before long navigation tasks
- Don't navigate to unknown locations

CURRENT ROBOT STATE:
{context}

OUTPUT FORMAT:
Respond with a JSON array of actions. Example:
[
  {"action": "navigate", "params": {"location": "kitchen"}, "description": "Going to kitchen"},
  {"action": "look", "params": {"object": "red cup"}, "description": "Looking for red cup"},
  {"action": "pick", "params": {"object": "cup", "color": "red"}, "description": "Picking up red cup"}
]

If the command is unsafe or impossible, respond:
{"error": "explanation", "suggestion": "alternative action if any"}
"""
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: str = "gpt-4-turbo-preview"
    ):
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        self.model = model
        self.client = AsyncOpenAI(api_key=self.api_key)
        self.context = RobotContext()
    
    def update_context(self, **kwargs):
        """Update robot context"""
        for key, value in kwargs.items():
            if hasattr(self.context, key):
                setattr(self.context, key, value)
    
    async def plan(
        self,
        command: str,
        context: Optional[RobotContext] = None
    ) -> List[RobotAction]:
        """
        Convert natural language command to action sequence.
        
        Args:
            command: Natural language command from user
            context: Optional robot context (uses stored if not provided)
            
        Returns:
            List of RobotAction objects
        """
        ctx = context or self.context
        
        # Build prompt
        system_prompt = self.SYSTEM_PROMPT.format(
            context=json.dumps(ctx.to_dict(), indent=2)
        )
        
        # Call LLM
        response = await self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1,  # Low temperature for consistent output
            max_tokens=1000
        )
        
        # Parse response
        content = response.choices[0].message.content
        return self._parse_response(content)
    
    def _parse_response(self, content: str) -> List[RobotAction]:
        """Parse LLM response into RobotAction objects"""
        try:
            # Try to extract JSON from response
            content = content.strip()
            
            # Handle markdown code blocks
            if "```json" in content:
                content = content.split("```json")[1].split("```")[0]
            elif "```" in content:
                content = content.split("```")[1].split("```")[0]
            
            data = json.loads(content)
            
            # Check for error response
            if isinstance(data, dict) and "error" in data:
                raise ValueError(data["error"])
            
            # Parse action list
            actions = []
            for item in data:
                action = RobotAction(
                    action_type=ActionType(item["action"]),
                    parameters=item.get("params", {}),
                    description=item.get("description", "")
                )
                actions.append(action)
            
            return actions
            
        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse LLM response: {e}")
    
    def validate_actions(
        self,
        actions: List[RobotAction]
    ) -> List[str]:
        """
        Validate action sequence for safety.
        
        Returns list of warnings/errors.
        """
        issues = []
        
        dangerous_objects = ["knife", "scissors", "glass", "hot", "fire"]
        
        for i, action in enumerate(actions):
            # Check navigation targets
            if action.action_type == ActionType.NAVIGATE:
                location = action.parameters.get("location", "")
                if location not in self.context.known_locations:
                    issues.append(f"Action {i}: Unknown location '{location}'")
            
            # Check pick targets
            if action.action_type == ActionType.PICK:
                obj = action.parameters.get("object", "").lower()
                for danger in dangerous_objects:
                    if danger in obj:
                        issues.append(f"Action {i}: Dangerous object '{obj}'")
            
            # Check battery for navigation
            if action.action_type == ActionType.NAVIGATE:
                if self.context.battery_level < 20:
                    issues.append(f"Action {i}: Low battery warning")
        
        return issues


# Example usage
async def main():
    """Test the task planner"""
    planner = TaskPlanner()
    
    # Update context
    planner.update_context(
        location="living_room",
        nearby_objects=["remote", "book"],
        battery_level=85
    )
    
    # Test command
    command = "Go to the kitchen and bring me a cup of water"
    
    try:
        actions = await planner.plan(command)
        
        print(f"Command: {command}")
        print(f"Actions:")
        for i, action in enumerate(actions):
            print(f"  {i+1}. {action.action_type.value}: {action.description}")
            print(f"      Params: {action.parameters}")
        
        # Validate
        issues = planner.validate_actions(actions)
        if issues:
            print(f"\nWarnings:")
            for issue in issues:
                print(f"  ⚠️ {issue}")
                
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
