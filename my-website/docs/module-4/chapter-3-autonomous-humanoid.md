---
title: Autonomous Humanoid Capstone
sidebar_position: 4
tags: [autonomous-robotics, humanoid, full-pipeline, command-processing, navigation, perception, manipulation, vla-integration]
description: Learn how to integrate all VLA components into a complete autonomous humanoid system that processes commands from start to finish with navigation, perception, and manipulation.
---

# Autonomous Humanoid Capstone

This capstone chapter brings together all the components of the Vision-Language-Action (VLA) framework into a complete autonomous humanoid system. We'll integrate voice command processing, cognitive planning with LLMs, navigation, perception, and manipulation into a cohesive system that can understand high-level goals and execute them autonomously.

## Overview of Autonomous Humanoid Systems

Autonomous humanoid systems represent the pinnacle of robotics integration, requiring seamless coordination between multiple complex subsystems:

- **Command Processing**: Understanding and interpreting high-level goals from natural language
- **Cognitive Planning**: Decomposing complex goals into executable action sequences
- **Navigation**: Moving the humanoid robot through the environment safely and efficiently
- **Perception**: Sensing and understanding the environment in real-time
- **Manipulation**: Interacting with objects in the environment to achieve goals
- **Adaptive Control**: Adjusting behavior based on environmental changes and feedback

This chapter demonstrates how to integrate these components into a unified system.

## Full VLA Pipeline Integration

### System Architecture

The complete VLA pipeline follows this flow:

```
Voice Command → Speech Recognition → Intent Classification → Action Planning →
Navigation → Perception → Manipulation → Execution Feedback → Goal Achievement
```

Each component must work harmoniously to achieve autonomous behavior:

```python
import asyncio
import threading
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum

class ExecutionState(Enum):
    IDLE = "idle"
    PROCESSING_COMMAND = "processing_command"
    PLANNING_ACTIONS = "planning_actions"
    EXECUTING_NAVIGATION = "executing_navigation"
    EXECUTING_PERCEPTION = "executing_perception"
    EXECUTING_MANIPULATION = "executing_manipulation"
    ADAPTIVE_PLANNING = "adaptive_planning"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class CommandContext:
    """
    Context for command execution including environment and robot state
    """
    robot_position: Dict[str, float]
    environment_map: Dict[str, Any]
    known_objects: List[Dict[str, Any]]
    robot_capabilities: Dict[str, bool]
    recent_interactions: List[Dict[str, Any]]

@dataclass
class ExecutionPlan:
    """
    A complete plan for executing a command
    """
    plan_id: str
    command: str
    action_sequence: List[Dict[str, Any]]
    estimated_duration: float
    confidence: float

class AutonomousHumanoidSystem:
    def __init__(self,
                 whisper_recognizer: 'WhisperVoiceRecognizer',
                 llm_planner: 'LLMBehaviorPlanner',
                 action_generator: 'ROS2ActionGenerator'):
        self.whisper_recognizer = whisper_recognizer
        self.llm_planner = llm_planner
        self.action_generator = action_generator

        self.current_state = ExecutionState.IDLE
        self.active_plan = None
        self.context = CommandContext(
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
            environment_map={},
            known_objects=[],
            robot_capabilities={
                "mobility": True,
                "manipulation": True,
                "perception": True,
                "communication": True
            },
            recent_interactions=[]
        )

        # Event handlers for state changes
        self.state_change_handlers = []
        self.progress_callbacks = []

    async def process_high_level_goal(self, command: str) -> bool:
        """
        Process a high-level goal and execute it autonomously
        """
        self._update_state(ExecutionState.PROCESSING_COMMAND)

        # Step 1: If command is spoken, convert to text
        if self._is_audio_command(command):
            audio_path = command  # Assume command is an audio path for this example
            recognition_result = await self.whisper_recognizer.recognize_speech(audio_path)
            command_text = recognition_result["text"]
            confidence = recognition_result["confidence"]

            if confidence < 0.6:
                self._log_error(f"Low confidence recognition: {confidence}")
                return False
        else:
            command_text = command

        self._log_info(f"Processing command: {command_text}")

        # Step 2: Update context with current state
        await self._update_context()

        # Step 3: Plan actions using LLM
        self._update_state(ExecutionState.PLANNING_ACTIONS)
        plan = await self.llm_planner.plan_behavior(
            command_text,
            self.context.robot_capabilities,
            {
                "robot_position": self.context.robot_position,
                "environment_map": self.context.environment_map,
                "known_objects": self.context.known_objects
            },
            self.context.recent_interactions
        )

        if not plan["action_sequence"]:
            self._log_error("No valid action sequence generated")
            self._update_state(ExecutionState.FAILED)
            return False

        # Create execution plan
        self.active_plan = ExecutionPlan(
            plan_id=plan["plan_id"],
            command=command_text,
            action_sequence=plan["action_sequence"],
            estimated_duration=plan.get("estimated_duration", 0.0),
            confidence=plan.get("confidence", 0.0)
        )

        # Step 4: Execute the plan
        success = await self._execute_plan(self.active_plan)

        if success:
            self._update_state(ExecutionState.COMPLETED)
            self._log_info("Goal achieved successfully")
        else:
            self._update_state(ExecutionState.FAILED)
            self._log_error("Goal execution failed")

        return success

    async def _execute_plan(self, plan: ExecutionPlan) -> bool:
        """
        Execute a complete action plan with monitoring and adaptation
        """
        self._log_info(f"Starting execution of plan: {plan.plan_id}")

        for i, action in enumerate(plan.action_sequence):
            self._log_info(f"Executing action {i+1}/{len(plan.action_sequence)}: {action['action_type']}")

            # Update state based on action type
            if action['action_type'] == 'NAVIGATION':
                self._update_state(ExecutionState.EXECUTING_NAVIGATION)
            elif action['action_type'] == 'PERCEPTION':
                self._update_state(ExecutionState.EXECUTING_PERCEPTION)
            elif action['action_type'] == 'MANIPULATION':
                self._update_state(ExecutionState.EXECUTING_MANIPULATION)

            # Execute the action
            success = await self._execute_single_adaptive_action(action)

            if not success:
                self._log_warning(f"Action {i+1} failed: {action}")

                # Attempt recovery or adaptive planning
                recovered = await self._attempt_recovery(plan, i, action)
                if not recovered:
                    return False

            # Update context after each action
            await self._update_context()

            # Check for environmental changes that might require replanning
            if await self._needs_adaptive_planning():
                self._update_state(ExecutionState.ADAPTIVE_PLANNING)
                plan = await self._adaptive_replan(plan, i)

        return True

    async def _execute_single_adaptive_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action with real-time adaptation
        """
        try:
            # Execute the action using the action generator
            success = await self.action_generator.execute_single_action(action)

            # If successful, update context and return
            if success:
                await self._update_context()
                return True

            # If failed, try adaptive strategies
            return await self._adaptive_execution_strategy(action)

        except Exception as e:
            self._log_error(f"Error executing action: {str(e)}")
            return False

    async def _attempt_recovery(self, plan: ExecutionPlan, failed_step: int, failed_action: Dict[str, Any]) -> bool:
        """
        Attempt to recover from a failed action
        """
        self._log_warning(f"Attempting recovery from failed action: {failed_action}")

        # Check if this is a recurring failure
        if self._is_recurring_failure(failed_action):
            # Try completely different approach
            return await self._alternative_approach(plan, failed_step, failed_action)

        # Try with modified parameters
        return await self._retry_with_modifications(plan, failed_step, failed_action)

    async def _needs_adaptive_planning(self) -> bool:
        """
        Check if environmental changes require replanning
        """
        # In a real system, this would check:
        # - Object positions have changed
        # - New obstacles appeared
        # - Robot state changed significantly
        # - User context changed

        # For this example, we'll simulate occasional environmental changes
        import random
        return random.random() < 0.1  # 10% chance of environmental change

    async def _adaptive_replan(self, original_plan: ExecutionPlan, current_step: int) -> ExecutionPlan:
        """
        Replan based on changed environment
        """
        remaining_command = self._extract_remaining_command(original_plan, current_step)

        # Get updated context
        await self._update_context()

        # Generate new plan for remaining command
        new_plan = await self.llm_planner.plan_behavior(
            remaining_command,
            self.context.robot_capabilities,
            {
                "robot_position": self.context.robot_position,
                "environment_map": self.context.environment_map,
                "known_objects": self.context.known_objects
            }
        )

        return ExecutionPlan(
            plan_id=new_plan["plan_id"],
            command=remaining_command,
            action_sequence=new_plan["action_sequence"],
            estimated_duration=new_plan.get("estimated_duration", 0.0),
            confidence=new_plan.get("confidence", 0.0)
        )

    def _update_state(self, new_state: ExecutionState):
        """
        Update the system state and notify listeners
        """
        old_state = self.current_state
        self.current_state = new_state

        # Notify state change handlers
        for handler in self.state_change_handlers:
            handler(old_state, new_state)

    async def _update_context(self):
        """
        Update the command context with current state information
        """
        # In a real system, this would:
        # - Query robot position from localization system
        # - Update environment map from SLAM
        # - Refresh known objects from perception system
        # - Update robot capabilities from diagnostics

        # For this example, we'll simulate updating
        self.context.robot_position = {
            "x": self.context.robot_position["x"] + (random.uniform(-0.1, 0.1) if random.random() > 0.7 else 0),
            "y": self.context.robot_position["y"] + (random.uniform(-0.1, 0.1) if random.random() > 0.7 else 0),
            "z": 0.0
        }

        # Add to recent interactions
        self.context.recent_interactions.append({
            "timestamp": time.time(),
            "event": "context_update",
            "robot_position": self.context.robot_position.copy()
        })

        # Limit history size
        if len(self.context.recent_interactions) > 20:
            self.context.recent_interactions = self.context.recent_interactions[-20:]

    def _is_audio_command(self, command: str) -> bool:
        """
        Check if command is an audio path
        """
        return command.endswith(('.wav', '.mp3', '.flac', '.ogg'))

    def _log_info(self, message: str):
        """
        Log informational message
        """
        print(f"[INFO] {message}")

    def _log_warning(self, message: str):
        """
        Log warning message
        """
        print(f"[WARNING] {message}")

    def _log_error(self, message: str):
        """
        Log error message
        """
        print(f"[ERROR] {message}")
```

## End-to-End Command Processing Workflows

### Complete Processing Pipeline

The end-to-end workflow for processing high-level commands involves multiple stages:

```python
class EndToEndProcessor:
    """
    Complete end-to-end processor for autonomous humanoid commands
    """
    def __init__(self, autonomous_system: AutonomousHumanoidSystem):
        self.system = autonomous_system
        self.pipeline_monitor = PipelineMonitor()

    async def process_command_pipeline(self, raw_command: str) -> Dict[str, Any]:
        """
        Complete pipeline: Raw command → Processed result
        """
        result = {
            "success": False,
            "stages": [],
            "execution_time": 0.0,
            "confidence": 0.0,
            "error": None
        }

        start_time = time.time()

        try:
            # Stage 1: Command Reception
            self.pipeline_monitor.start_stage("reception")
            processed_command = await self._receive_and_validate(raw_command)
            result["stages"].append("reception")
            self.pipeline_monitor.end_stage("reception")

            # Stage 2: Context Acquisition
            self.pipeline_monitor.start_stage("context_acquisition")
            await self.system._update_context()
            result["stages"].append("context_acquisition")
            self.pipeline_monitor.end_stage("context_acquisition")

            # Stage 3: Planning
            self.pipeline_monitor.start_stage("planning")
            plan_success = await self.system.process_high_level_goal(processed_command)
            result["stages"].append("planning")
            self.pipeline_monitor.end_stage("planning")

            # Stage 4: Execution
            self.pipeline_monitor.start_stage("execution")
            if plan_success:
                result["success"] = True
                result["confidence"] = self.system.active_plan.confidence if self.system.active_plan else 0.0
            result["stages"].append("execution")
            self.pipeline_monitor.end_stage("execution")

        except Exception as e:
            result["error"] = str(e)
            self.system._log_error(f"Pipeline error: {str(e)}")
        finally:
            result["execution_time"] = time.time() - start_time
            result["metrics"] = self.pipeline_monitor.get_metrics()

        return result

    async def _receive_and_validate(self, raw_command: str) -> str:
        """
        Receive and validate raw command
        """
        # Validate command format
        if not raw_command or len(raw_command.strip()) < 3:
            raise ValueError("Command too short")

        # If it's an audio command, validate format
        if self.system._is_audio_command(raw_command):
            if not os.path.exists(raw_command):
                raise FileNotFoundError(f"Audio file not found: {raw_command}")

        return raw_command

class PipelineMonitor:
    """
    Monitor pipeline performance and metrics
    """
    def __init__(self):
        self.stages = {}
        self.current_stage_start = {}

    def start_stage(self, stage_name: str):
        """
        Start timing a stage
        """
        self.current_stage_start[stage_name] = time.time()

    def end_stage(self, stage_name: str):
        """
        End timing a stage
        """
        if stage_name in self.current_stage_start:
            duration = time.time() - self.current_stage_start[stage_name]
            if stage_name not in self.stages:
                self.stages[stage_name] = []
            self.stages[stage_name].append(duration)

    def get_metrics(self) -> Dict[str, Any]:
        """
        Get pipeline metrics
        """
        metrics = {}
        for stage_name, durations in self.stages.items():
            if durations:
                metrics[f"{stage_name}_avg_time"] = sum(durations) / len(durations)
                metrics[f"{stage_name}_total_time"] = sum(durations)
                metrics[f"{stage_name}_call_count"] = len(durations)

        return metrics
```

## Adaptive Planning and Error Recovery

### Dynamic Plan Adjustment

Autonomous systems must be able to adapt to changing conditions:

```python
class AdaptivePlanner:
    """
    Handles dynamic plan adjustment based on environmental changes
    """
    def __init__(self, base_planner: 'LLMBehaviorPlanner'):
        self.base_planner = base_planner
        self.plan_history = []
        self.failure_patterns = {}

    async def adapt_plan(self,
                        original_plan: ExecutionPlan,
                        current_context: Dict[str, Any],
                        reason_for_adaptation: str) -> ExecutionPlan:
        """
        Adapt the current plan based on new information
        """
        adaptation_prompt = f"""
        The original plan needs to be adapted due to: {reason_for_adaptation}

        Original command: "{original_plan.command}"
        Original plan: {json.dumps(original_plan.action_sequence, indent=2)}

        Current context:
        - Robot position: {current_context.get('robot_position', {})}
        - Environment: {current_context.get('environment_map', {})}
        - Known objects: {current_context.get('known_objects', [])}

        Generate an adapted plan that accounts for the changes while still achieving the goal.
        Respond in the same JSON format as the original plan.
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.base_planner.model,
                messages=[
                    {"role": "system", "content": "You are an expert in adapting robot plans to changing conditions."},
                    {"role": "user", "content": adaptation_prompt}
                ],
                temperature=0.3
            )

            adapted_plan_data = json.loads(response.choices[0].message.content)

            # Create new plan with adapted sequence
            adapted_plan = ExecutionPlan(
                plan_id=f"adapted_{original_plan.plan_id}_{int(time.time())}",
                command=original_plan.command,
                action_sequence=adapted_plan_data.get("action_sequence", []),
                estimated_duration=adapted_plan_data.get("estimated_duration", original_plan.estimated_duration),
                confidence=adapted_plan_data.get("confidence", original_plan.confidence * 0.9)  # Slightly lower confidence for adaptations
            )

            # Store in history
            self.plan_history.append({
                "original_plan_id": original_plan.plan_id,
                "adapted_plan": adapted_plan,
                "adaptation_reason": reason_for_adaptation,
                "timestamp": time.time()
            })

            return adapted_plan

        except Exception as e:
            print(f"Error adapting plan: {str(e)}")
            return original_plan  # Return original if adaptation fails

    def learn_from_failures(self, failed_plan: ExecutionPlan, failure_reason: str):
        """
        Learn from plan failures to improve future planning
        """
        if failure_reason not in self.failure_patterns:
            self.failure_patterns[failure_reason] = {
                "count": 0,
                "plans_affected": []
            }

        self.failure_patterns[failure_reason]["count"] += 1
        self.failure_patterns[failure_reason]["plans_affected"].append(failed_plan.plan_id)

    def get_adaptation_recommendations(self, current_context: Dict[str, Any]) -> List[str]:
        """
        Get recommendations for plan adaptation based on learned patterns
        """
        recommendations = []

        # Check for known failure patterns that might apply
        for pattern, data in self.failure_patterns.items():
            if data["count"] > 3:  # If this pattern occurred more than 3 times
                recommendations.append(f"Avoid pattern: {pattern} (occurred {data['count']} times)")

        return recommendations
```

## Practical Examples of Autonomous Humanoid Implementations

### Example 1: Office Cleaning Assistant

```python
class OfficeCleaningAssistant:
    """
    Example implementation: Autonomous office cleaning
    """
    def __init__(self, autonomous_system: AutonomousHumanoidSystem):
        self.system = autonomous_system

    async def execute_cleaning_task(self, command: str = "Clean up the office") -> bool:
        """
        Execute an office cleaning task
        """
        # Enhanced command with context
        enhanced_command = f"""
        {command}. Specifically:
        - Collect all trash items and dispose of them in the waste bin
        - Organize scattered papers on desks
        - Return misplaced items to their proper locations
        - Navigate safely around people and furniture
        - Use gentle manipulation to avoid damage
        """

        # Update context with office-specific information
        self.system.context.environment_map.update({
            "trash_bins": [{"location": {"x": 5.0, "y": 3.0}, "type": "waste"}],
            "desks": [{"location": {"x": 2.0, "y": 1.0}, "area": "workspace_1"}],
            "corridors": [{"start": {"x": 0, "y": 0}, "end": {"x": 10, "y": 0}}],
            "charging_station": {"location": {"x": -2.0, "y": -2.0}}
        })

        # Execute the cleaning task
        success = await self.system.process_high_level_goal(enhanced_command)

        if success:
            # Return to charging station
            return await self._return_to_charging()

        return success

    async def _return_to_charging(self) -> bool:
        """
        Return to charging station after task completion
        """
        charge_command = "Navigate to the charging station at coordinates x=-2.0, y=-2.0"
        return await self.system.process_high_level_goal(charge_command)
```

### Example 2: Home Assistance Robot

```python
class HomeAssistanceRobot:
    """
    Example implementation: Home assistance for elderly care
    """
    def __init__(self, autonomous_system: AutonomousHumanoidSystem):
        self.system = autonomous_system
        self.resident_profile = {
            "preferences": {"volume": 0.7, "speaking_speed": "normal"},
            "schedule": {"medication_times": ["08:00", "12:00", "18:00"]},
            "safe_zones": ["living_room", "kitchen", "bedroom"],
            "forbidden_areas": ["attic", "basement_when_dark"]
        }

    async def execute_daily_assistance(self) -> bool:
        """
        Execute daily assistance tasks
        """
        # Morning routine
        morning_tasks = [
            "Check if the resident is awake",
            "Prepare a glass of water and bring it to the bedroom",
            "Announce the weather and today's schedule",
            "Check medication schedule and remind about 8 AM dose"
        ]

        all_success = True
        for task in morning_tasks:
            success = await self.system.process_high_level_goal(task)
            all_success = all_success and success
            if not success:
                self.system._log_warning(f"Task failed: {task}")

        return all_success

    async def emergency_response(self, emergency_type: str) -> bool:
        """
        Handle emergency situations
        """
        if emergency_type == "medical_emergency":
            command = "Immediately go to the resident's location and call for medical assistance"
        elif emergency_type == "fall_detection":
            command = "Go to the detected fall location, assess situation, and call for help"
        elif emergency_type == "fire_alarm":
            command = "Evacuate the resident to the predetermined safe location"
        else:
            command = f"Handle {emergency_type} situation according to protocol"

        # Execute emergency command with highest priority
        self.system.context.robot_capabilities["priority"] = "emergency"
        return await self.system.process_high_level_goal(command)
```

### Example 3: Retail Customer Service Robot

```python
class RetailCustomerServiceRobot:
    """
    Example implementation: Customer service in retail environment
    """
    def __init__(self, autonomous_system: AutonomousHumanoidSystem):
        self.system = autonomous_system
        self.store_layout = {
            "departments": {
                "electronics": {"section_id": "ELEC001", "coordinates": {"x": 10, "y": 5}},
                "clothing": {"section_id": "CLOT001", "coordinates": {"x": 15, "y": 8}},
                "checkout": {"section_id": "CHK001", "coordinates": {"x": 20, "y": 2}}
            },
            "product_locations": {
                "smartphone": {"department": "electronics", "aisle": "A1", "shelf": "top"},
                "dress": {"department": "clothing", "aisle": "B2", "rack": "left"}
            },
            "customer_flow": ["entrance", "electronics", "clothing", "checkout"]
        }

    async def assist_customer(self, customer_request: str) -> bool:
        """
        Assist customers with various requests
        """
        # Enrich the request with store context
        contextualized_request = f"""
        {customer_request}. Use your knowledge of the store layout:
        - Departments: {list(self.store_layout['departments'].keys())}
        - Product locations: {list(self.store_layout['product_locations'].keys())[:5]} (showing first 5)
        - Typical customer flow: {self.store_layout['customer_flow']}

        Provide helpful guidance and navigation assistance.
        """

        # Update context with store information
        self.system.context.environment_map.update(self.store_layout)

        return await self.system.process_high_level_goal(contextualized_request)

    async def restock_assistant(self) -> bool:
        """
        Assist with restocking shelves
        """
        # Get inventory alerts
        inventory_alerts = self._get_inventory_alerts()

        for alert in inventory_alerts:
            if alert["low_stock"]:
                product = alert["product"]
                location = self.store_layout["product_locations"].get(product, {})

                if location:
                    command = f"Navigate to {location['department']} department, aisle {location.get('aisle', 'unknown')}, " \
                             f"and check the stock level of {product}. Report findings."

                    success = await self.system.process_high_level_goal(command)
                    if not success:
                        self.system._log_error(f"Restock check failed for {product}")

        return True

    def _get_inventory_alerts(self) -> List[Dict[str, Any]]:
        """
        Get inventory alerts from store system (simulated)
        """
        # Simulated inventory alerts
        import random
        alerts = []

        if random.random() > 0.7:  # 30% chance of an alert
            alerts.append({
                "product": "smartphone",
                "low_stock": True,
                "current_quantity": 2,
                "reorder_threshold": 5
            })

        return alerts
```

## Configuration Examples for Complete VLA Systems

### System Configuration

```python
# autonomous_config.py
class AutonomousConfig:
    # Core system parameters
    SYSTEM_NAME = os.getenv('SYSTEM_NAME', 'AutonomousHumanoid')
    ROBOT_TYPE = os.getenv('ROBOT_TYPE', 'humanoid')

    # Processing parameters
    CONFIDENCE_THRESHOLD = float(os.getenv('CONFIDENCE_THRESHOLD', 0.7))
    MAX_EXECUTION_TIME = int(os.getenv('MAX_EXECUTION_TIME', 300))  # 5 minutes
    MAX_PLAN_STEPS = int(os.getenv('MAX_PLAN_STEPS', 50))

    # Adaptation parameters
    ENVIRONMENT_CHANGE_THRESHOLD = float(os.getenv('ENVIRONMENT_CHANGE_THRESHOLD', 0.1))
    ADAPTATION_INTERVAL = int(os.getenv('ADAPTATION_INTERVAL', 30))  # seconds

    # Recovery parameters
    MAX_RECOVERY_ATTEMPTS = int(os.getenv('MAX_RECOVERY_ATTEMPTS', 3))
    RECOVERY_DELAY = float(os.getenv('RECOVERY_DELAY', 2.0))

    # Safety parameters
    SAFETY_TIMEOUT = int(os.getenv('SAFETY_TIMEOUT', 60))
    EMERGENCY_STOP_ENABLED = os.getenv('EMERGENCY_STOP_ENABLED', 'true').lower() == 'true'
    SAFE_ZONES_ONLY = os.getenv('SAFE_ZONES_ONLY', 'false').lower() == 'true'

    # Component-specific configs
    class Whisper:
        MODEL = os.getenv('WHISPER_MODEL', 'whisper-1')
        CONFIDENCE_THRESHOLD = float(os.getenv('WHISPER_CONFIDENCE', 0.6))
        LANGUAGE = os.getenv('WHISPER_LANGUAGE', 'en')

    class LLM:
        MODEL = os.getenv('LLM_MODEL', 'gpt-4')
        TEMPERATURE = float(os.getenv('LLM_TEMPERATURE', 0.2))
        MAX_TOKENS = int(os.getenv('LLM_MAX_TOKENS', 1000))
        PLANNING_CONFIDENCE = float(os.getenv('LLM_PLANNING_CONFIDENCE', 0.7))

    class Navigation:
        MAX_SPEED = float(os.getenv('NAV_MAX_SPEED', 0.5))
        MIN_DISTANCE_TO_OBSTACLE = float(os.getenv('NAV_MIN_OBSTACLE_DIST', 0.3))
        RECOVERY_BEHAVIORS = os.getenv('NAV_RECOVERY', 'true').lower() == 'true'
```

### Runtime Configuration

```python
# runtime_config.py
class RuntimeProfile:
    """
    Different operational profiles for different scenarios
    """
    PROFILES = {
        "conservative": {
            "confidence_multiplier": 1.2,  # Require higher confidence
            "execution_speed": 0.7,  # Slower execution
            "planning_horizon": 5,  # Shorter term planning
            "safety_margin": 1.5,  # Wider safety margins
            "verification_steps": True  # More verification
        },
        "aggressive": {
            "confidence_multiplier": 0.8,  # Lower confidence requirement
            "execution_speed": 1.5,  # Faster execution
            "planning_horizon": 15,  # Longer term planning
            "safety_margin": 0.8,  # Tighter margins
            "verification_steps": False  # Fewer verifications
        },
        "standard": {
            "confidence_multiplier": 1.0,
            "execution_speed": 1.0,
            "planning_horizon": 10,
            "safety_margin": 1.0,
            "verification_steps": True
        },
        "emergency": {
            "confidence_multiplier": 0.5,  # Lower threshold for urgent responses
            "execution_speed": 2.0,  # Maximum speed
            "planning_horizon": 3,  # Immediate actions only
            "safety_margin": 0.5,  # Minimal safety for emergencies
            "verification_steps": False,
            "priority": "highest"
        }
    }

def get_runtime_config(profile_name: str = "standard") -> Dict[str, Any]:
    """
    Get configuration based on operational profile
    """
    base_config = vars(AutonomousConfig)
    profile_config = RuntimeProfile.PROFILES.get(profile_name, RuntimeProfile.PROFILES["standard"])

    # Merge configurations
    merged_config = {**base_config, **profile_config}
    merged_config["profile"] = profile_name

    return merged_config
```

## Code Snippets for Integrated VLA Systems

### Main System Orchestrator

```python
import signal
import sys
import asyncio
from concurrent.futures import ThreadPoolExecutor

class VLAOrchestrator:
    """
    Main orchestrator for the complete VLA system
    """
    def __init__(self, config: AutonomousConfig):
        self.config = config
        self.is_running = False
        self.command_queue = asyncio.Queue()
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Initialize components
        self.whisper_recognizer = WhisperVoiceRecognizer(api_key=config.LLM_MODEL)
        self.llm_planner = LLMBehaviorPlanner(api_key=config.LLM_MODEL, model=config.LLM.MODEL)
        self.action_generator = ROS2ActionGenerator()  # Assuming ROS2 node
        self.autonomous_system = AutonomousHumanoidSystem(
            self.whisper_recognizer,
            self.llm_planner,
            self.action_generator
        )

        # Register shutdown handler
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    async def start(self):
        """
        Start the autonomous system
        """
        self.is_running = True
        self.autonomous_system._log_info("Starting VLA Orchestrator...")

        # Start the main processing loop
        await self._main_processing_loop()

    async def _main_processing_loop(self):
        """
        Main processing loop that handles commands and system state
        """
        self.autonomous_system._log_info("VLA Orchestrator running. Waiting for commands...")

        while self.is_running:
            try:
                # Check for new commands
                if not self.command_queue.empty():
                    command = await self.command_queue.get()
                    await self._process_command(command)

                # Perform periodic system checks
                await self._perform_system_health_check()

                # Small delay to prevent busy waiting
                await asyncio.sleep(0.1)

            except Exception as e:
                self.autonomous_system._log_error(f"Error in main loop: {str(e)}")
                await asyncio.sleep(1)  # Brief pause before continuing

    async def _process_command(self, command: str):
        """
        Process a single command through the VLA pipeline
        """
        try:
            self.autonomous_system._log_info(f"Processing command: {command[:50]}...")

            # Process through end-to-end pipeline
            processor = EndToEndProcessor(self.autonomous_system)
            result = await processor.process_command_pipeline(command)

            if result["success"]:
                self.autonomous_system._log_info(f"Command completed successfully: {command[:30]}...")
            else:
                self.autonomous_system._log_error(f"Command failed: {result['error']}")

        except Exception as e:
            self.autonomous_system._log_error(f"Command processing error: {str(e)}")

    async def _perform_system_health_check(self):
        """
        Perform periodic health checks
        """
        # Check if all components are responsive
        # In a real system, this would check:
        # - LLM API connectivity
        # - ROS2 node status
        # - Sensor availability
        # - Battery level
        # - Navigation system status

        # For this example, just log periodically
        if int(time.time()) % 30 == 0:  # Every 30 seconds
            self.autonomous_system._log_info(f"System health check - State: {self.autonomous_system.current_state.value}")

    def add_command(self, command: str):
        """
        Add a command to the processing queue
        """
        asyncio.create_task(self.command_queue.put(command))

    def _signal_handler(self, signum, frame):
        """
        Handle shutdown signals
        """
        self.autonomous_system._log_info(f"Received signal {signum}, shutting down...")
        self.is_running = False
        self.executor.shutdown(wait=True)
        sys.exit(0)

    async def shutdown(self):
        """
        Gracefully shut down the system
        """
        self.is_running = False
        self.executor.shutdown(wait=True)
        self.autonomous_system._log_info("VLA Orchestrator shut down gracefully.")
```

### Command Interface

```python
class CommandInterface:
    """
    Interface for accepting commands from various sources
    """
    def __init__(self, orchestrator: VLAOrchestrator):
        self.orchestrator = orchestrator

    def accept_voice_command(self, audio_path: str):
        """
        Accept voice command from audio file
        """
        self.orchestrator.add_command(audio_path)

    def accept_text_command(self, text: str):
        """
        Accept text command directly
        """
        self.orchestrator.add_command(text)

    def accept_multimodal_command(self, text: str, context_data: Dict[str, Any] = None):
        """
        Accept command with additional context data
        """
        # Combine text with context information
        if context_data:
            command_with_context = {
                "command": text,
                "context": context_data
            }
            # For now, just pass the text - context would be handled by the system
            self.orchestrator.add_command(text)
        else:
            self.orchestrator.add_command(text)

# Example usage
async def main():
    """
    Example main function showing how to use the system
    """
    config = AutonomousConfig()
    orchestrator = VLAOrchestrator(config)
    interface = CommandInterface(orchestrator)

    try:
        # Add some example commands
        interface.accept_text_command("Clean up the workspace by organizing papers and disposing of trash")
        interface.accept_text_command("Go to the kitchen and bring me a glass of water")
        interface.accept_text_command("Find my keys and bring them to me")

        # Start the orchestrator
        await orchestrator.start()

    except KeyboardInterrupt:
        print("\nShutting down...")
        await orchestrator.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
```

## Exercises for Full System Integration

### Exercise 1: Complete System Integration
Integrate all VLA components into a working autonomous humanoid system.

**Requirements**:
- Integrate voice recognition, LLM planning, and action execution
- Create a complete pipeline from voice command to robot action
- Implement basic error handling and recovery
- Test with simple commands like "move forward" or "turn left"

**Steps**:
1. Implement the AutonomousHumanoidSystem class
2. Connect all components: voice → LLM → actions
3. Create a simple test environment
4. Test with basic commands

### Exercise 2: Multi-Step Task Execution
Extend the system to handle complex multi-step tasks.

**Requirements**:
- Execute commands that require multiple navigation, perception, and manipulation steps
- Maintain state between actions
- Handle intermediate failures and recovery
- Test with commands like "Go to the kitchen, find a red cup, and bring it to the living room"

**Implementation**:
- Implement the EndToEndProcessor
- Create state management between actions
- Add error recovery mechanisms
- Test with multi-step commands

### Exercise 3: Adaptive System Behavior
Implement adaptation to environmental changes.

**Requirements**:
- Detect when environmental conditions change during task execution
- Adjust plans dynamically based on new information
- Learn from failures to improve future performance
- Test with simulated environmental changes

**Components**:
- Environmental change detection
- Dynamic plan adaptation
- Failure learning and pattern recognition
- Continuous improvement mechanisms

## Summary

The Autonomous Humanoid Capstone chapter demonstrates the complete integration of the Vision-Language-Action framework into a unified autonomous system. By combining voice command processing, cognitive planning with LLMs, navigation, perception, and manipulation, we create a sophisticated system capable of understanding high-level goals and executing them autonomously.

The key components of the autonomous system include:
- Complete end-to-end command processing pipeline
- Adaptive planning and error recovery mechanisms
- Integration of all VLA components into a cohesive system
- Monitoring and health checking for reliable operation

This capstone implementation represents the culmination of the VLA module, showing how individual components work together to create intelligent, responsive robotic systems that can understand and execute natural language commands in complex environments. The system demonstrates the potential of combining modern AI techniques with robotics to create truly autonomous agents.