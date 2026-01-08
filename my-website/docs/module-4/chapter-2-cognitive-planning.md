---
title: Cognitive Planning with LLMs
sidebar_position: 3
tags: [llm, cognitive-planning, gpt, natural-language, ros2-actions, multi-step-planning]
description: Learn how to use Large Language Models for cognitive planning in robotics, decomposing complex natural language commands into sequences of ROS 2 actions.
---

# Cognitive Planning with LLMs

This chapter explores the use of Large Language Models (LLMs) for cognitive planning in robotics. We'll learn how to decompose complex natural language commands into sequences of ROS 2 actions, enabling robots to understand and execute sophisticated multi-step tasks. This represents the advanced cognitive capabilities that differentiate Vision-Language-Action (VLA) systems from simple command-response systems.

## Overview of Cognitive Planning in Robotics

Cognitive planning with LLMs enables robots to:

- Understand complex, multi-step natural language commands
- Decompose high-level goals into actionable steps
- Reason about the environment and available resources
- Generate executable action plans that achieve the desired outcome
- Handle ambiguity and provide feedback when clarification is needed

Unlike simple command mapping, cognitive planning involves higher-level reasoning that allows robots to adapt to different situations and contexts.

## LLM Integration for Cognitive Planning

### Choosing the Right LLM for Robotics

When selecting an LLM for cognitive planning in robotics, consider these factors:

#### Model Capabilities
- **Reasoning ability**: The model's capacity to understand and decompose complex tasks
- **Instruction following**: How well the model follows specific instructions and formatting
- **Knowledge cutoff**: The date of the model's training data and its relevance to current robotics knowledge
- **Context window**: The amount of information the model can process at once

#### Practical Considerations
- **Latency**: Response time is crucial for interactive applications
- **Cost**: API costs can add up quickly in production systems
- **Reliability**: Availability and consistency of the service
- **Safety**: The model's tendency to generate harmful or incorrect outputs

### Structured Prompting for Action Planning

Effective cognitive planning requires carefully crafted prompts that guide the LLM to generate structured outputs suitable for robot action execution:

```python
import openai
import json
from typing import Dict, List, Any, Optional

class LLMBehaviorPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model

    async def plan_behavior(self,
                          command: str,
                          robot_capabilities: Dict[str, Any],
                          environment_state: Dict[str, Any],
                          context_history: List[Dict[str, str]] = None) -> Dict[str, Any]:
        """
        Generate a sequence of actions from a natural language command using LLM
        """
        # Build context for the LLM
        context_prompt = self._build_context_prompt(
            command,
            robot_capabilities,
            environment_state,
            context_history
        )

        # Define the expected output format
        format_instruction = """
        Respond in the following JSON format:
        {
            "plan_id": "unique identifier for this plan",
            "command_understanding": "brief summary of what the user wants",
            "action_sequence": [
                {
                    "step": 1,
                    "action_type": "NAVIGATION | MANIPULATION | PERCEPTION | QUERY | WAIT",
                    "description": "what this step does",
                    "parameters": {
                        // specific parameters for the action type
                    },
                    "estimated_duration": "estimated time in seconds",
                    "confidence": "0.0-1.0 confidence in this step"
                }
            ],
            "potential_issues": ["list of potential problems"],
            "success_criteria": "how to verify the command was executed successfully"
        }
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": f"{context_prompt}\n\n{format_instruction}"}
                ],
                temperature=0.2,  # Low temperature for more consistent planning
                max_tokens=1000
            )

            # Parse the response
            result = json.loads(response.choices[0].message.content)

            # Validate the plan
            if not self._validate_action_sequence(result.get("action_sequence", [])):
                raise ValueError("Generated action sequence is invalid")

            return result

        except json.JSONDecodeError:
            return {
                "plan_id": "error",
                "command_understanding": "Could not parse LLM response",
                "action_sequence": [],
                "potential_issues": ["LLM response format was invalid"],
                "success_criteria": "No plan generated"
            }
        except Exception as e:
            return {
                "plan_id": "error",
                "command_understanding": f"Error generating plan: {str(e)}",
                "action_sequence": [],
                "potential_issues": [str(e)],
                "success_criteria": "No plan generated"
            }

    def _build_context_prompt(self,
                             command: str,
                             robot_capabilities: Dict[str, Any],
                             environment_state: Dict[str, Any],
                             context_history: List[Dict[str, str]] = None) -> str:
        """
        Build a comprehensive context prompt for the LLM
        """
        context_parts = []

        # Robot capabilities
        capabilities_desc = []
        if robot_capabilities.get("mobility", False):
            capabilities_desc.append("The robot can move around the environment")
        if robot_capabilities.get("manipulation", False):
            capabilities_desc.append("The robot can manipulate objects")
        if robot_capabilities.get("perception", False):
            capabilities_desc.append("The robot can perceive its environment")

        context_parts.append(f"Robot Capabilities: {', '.join(capabilities_desc)}")

        # Environment state
        if environment_state:
            context_parts.append(f"Environment State: {json.dumps(environment_state, indent=2)}")

        # Context history (for multi-turn interactions)
        if context_history:
            history_str = "\n".join([f"- {entry.get('role', 'user')}: {entry.get('content', '')}"
                                   for entry in context_history[-3:]])  # Last 3 exchanges
            context_parts.append(f"Recent Interaction History:\n{history_str}")

        # The actual command
        context_parts.append(f"User Command: '{command}'")

        return "\n\n".join(context_parts)

    def _get_system_prompt(self) -> str:
        """
        System prompt to guide the LLM's behavior planning
        """
        return """
        You are an expert robot behavior planner. Your job is to decompose natural language commands
        into executable action sequences for a robot. Consider:

        1. The robot's capabilities and limitations
        2. The current environment and its constraints
        3. The most efficient sequence of actions to achieve the goal
        4. Potential obstacles and how to handle them
        5. Safety considerations

        Always respond in the specified JSON format. Be precise and realistic about what the robot can do.
        """

    def _validate_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Validate that the action sequence is reasonable
        """
        if not action_sequence:
            return False

        for step in action_sequence:
            if not isinstance(step, dict):
                return False
            if "action_type" not in step or "parameters" not in step:
                return False
            if step["action_type"] not in ["NAVIGATION", "MANIPULATION", "PERCEPTION", "QUERY", "WAIT"]:
                return False

        return True
```

## Natural Language Decomposition into Action Sequences

### Understanding Command Complexity

Natural language commands can vary greatly in complexity. The LLM-based cognitive planner must handle:

#### Simple Commands
- "Move forward 1 meter"
- "Turn left"
- "Pick up the red block"

These require minimal decomposition and can often map directly to single robot actions.

#### Medium Complexity Commands
- "Go to the kitchen and bring me a cup"
- "Find the blue pen and put it in the drawer"

These require a sequence of 2-4 actions involving navigation, perception, and manipulation.

#### Complex Commands
- "Clean up the workspace by putting all the papers in the recycling bin and all the pens in the desk organizer"
- "Go to the meeting room, check if John is there, and if he is, tell him to join the conference call in 5 minutes"

These require sophisticated planning involving conditional logic, multiple subtasks, and context awareness.

### Decomposition Strategies

#### Sequential Decomposition
Break down commands into a linear sequence of steps:

```python
class SequentialDecomposer:
    """
    Decomposes commands into a linear sequence of actions
    """
    def __init__(self, llm_planner: LLMBehaviorPlanner):
        self.planner = llm_planner

    async def decompose(self, command: str, context: Dict[str, Any]) -> List[Dict[str, Any]]:
        # For simple sequential tasks, we can use direct LLM planning
        plan = await self.planner.plan_behavior(command, context["capabilities"], context["environment"])
        return plan["action_sequence"]
```

#### Hierarchical Decomposition
Break down complex commands into subgoals and then decompose each subgoal:

```python
class HierarchicalDecomposer:
    """
    Decomposes commands hierarchically into subgoals
    """
    def __init__(self, llm_planner: LLMBehaviorPlanner):
        self.planner = llm_planner

    async def decompose(self, command: str, context: Dict[str, Any]) -> List[Dict[str, Any]]:
        # First, ask the LLM to identify subgoals
        subgoal_prompt = f"""
        Decompose the following command into high-level subgoals:
        Command: "{command}"

        Respond in JSON format:
        {{
            "subgoals": [
                {{"id": 1, "description": "first subgoal"}},
                {{"id": 2, "description": "second subgoal"}}
            ]
        }}
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.planner.model,
                messages=[
                    {"role": "system", "content": "You are a task decomposition expert."},
                    {"role": "user", "content": subgoal_prompt}
                ],
                temperature=0.1
            )

            subgoals = json.loads(response.choices[0].message.content)["subgoals"]

            # Decompose each subgoal individually
            all_actions = []
            for subgoal in subgoals:
                subgoal_plan = await self.planner.plan_behavior(
                    subgoal["description"],
                    context["capabilities"],
                    context["environment"]
                )
                all_actions.extend(subgoal_plan["action_sequence"])

            return all_actions

        except Exception as e:
            # Fallback to direct decomposition
            return await self.direct_decompose(command, context)

    async def direct_decompose(self, command: str, context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Fallback to direct decomposition if hierarchical fails
        """
        plan = await self.planner.plan_behavior(command, context["capabilities"], context["environment"])
        return plan["action_sequence"]
```

## ROS 2 Action Plan Generation

### Converting LLM Output to Executable Actions

The cognitive planner must convert the LLM's structured output into actual ROS 2 action calls:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from typing import Dict, Any, List

class ROS2ActionGenerator(Node):
    def __init__(self):
        super().__init__('llm_action_generator')

        # Action clients for different types of actions
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        self.perception_client = ActionClient(self, PerceiveScene, 'perceive_scene')

        # Publishers for status updates
        self.status_pub = self.create_publisher(String, 'behavior_status', 10)

        self.get_logger().info("LLM Action Generator initialized")

    async def execute_action_sequence(self, action_sequence: List[Dict[str, Any]],
                                   success_callback=None, failure_callback=None):
        """
        Execute a sequence of actions generated by the LLM
        """
        for i, action in enumerate(action_sequence):
            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action['action_type']}")

            success = await self.execute_single_action(action)

            if not success:
                self.get_logger().error(f"Action {i+1} failed: {action}")
                if failure_callback:
                    await failure_callback(action_sequence, i, action)
                return False

            # Publish status update
            status_msg = String()
            status_msg.data = f"Completed action {i+1}/{len(action_sequence)}: {action['description']}"
            self.status_pub.publish(status_msg)

        self.get_logger().info("Action sequence completed successfully")
        if success_callback:
            await success_callback(action_sequence)

        return True

    async def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action based on its type
        """
        action_type = action.get('action_type', '').upper()

        if action_type == 'NAVIGATION':
            return await self.execute_navigation_action(action['parameters'])
        elif action_type == 'MANIPULATION':
            return await self.execute_manipulation_action(action['parameters'])
        elif action_type == 'PERCEPTION':
            return await self.execute_perception_action(action['parameters'])
        elif action_type == 'QUERY':
            return await self.execute_query_action(action['parameters'])
        elif action_type == 'WAIT':
            return await self.execute_wait_action(action['parameters'])
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            return False

    async def execute_navigation_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute navigation action
        """
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return False

        goal_msg = NavigateToPose.Goal()

        # Parse navigation parameters
        if 'position' in params:
            pos = params['position']
            goal_msg.pose.pose.position.x = pos.get('x', 0.0)
            goal_msg.pose.pose.position.y = pos.get('y', 0.0)
            goal_msg.pose.pose.position.z = pos.get('z', 0.0)

        if 'orientation' in params:
            orient = params['orientation']
            goal_msg.pose.pose.orientation.x = orient.get('x', 0.0)
            goal_msg.pose.pose.orientation.y = orient.get('y', 0.0)
            goal_msg.pose.pose.orientation.z = orient.get('z', 0.0)
            goal_msg.pose.pose.orientation.w = orient.get('w', 1.0)

        if 'frame_id' in params:
            goal_msg.pose.header.frame_id = params['frame_id']
        else:
            goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Send goal and wait for result
        goal_handle = await self.nav_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected")
            return False

        result = await goal_handle.get_result_async()

        return result.result.success  # Assuming the result has a success field

    async def execute_manipulation_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute manipulation action
        """
        if not self.manipulation_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Manipulation action server not available")
            return False

        goal_msg = ManipulateObject.Goal()

        # Parse manipulation parameters
        if 'object_id' in params:
            goal_msg.object_id = params['object_id']

        if 'action' in params:
            # Map natural language action to enum
            action_map = {
                'pick': ManipulateObject.PICK_UP,
                'place': ManipulateObject.PLACE_DOWN,
                'grasp': ManipulateObject.GRASP,
                'release': ManipulateObject.RELEASE
            }
            action_str = params['action'].lower()
            goal_msg.action = action_map.get(action_str, ManipulateObject.UNKNOWN)

        if 'target_position' in params:
            goal_msg.target_pose.position = Point(**params['target_position'])

        # Send goal and wait for result
        goal_handle = await self.manipulation_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error("Manipulation goal was rejected")
            return False

        result = await goal_handle.get_result_async()

        return result.result.success

    async def execute_perception_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute perception action
        """
        if not self.perception_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Perception action server not available")
            return False

        goal_msg = PerceiveScene.Goal()

        # Parse perception parameters
        if 'target_object' in params:
            goal_msg.target_object = params['target_object']

        if 'search_area' in params:
            # Assuming search_area has min/max coordinates
            goal_msg.search_area.min_corner.x = params['search_area'].get('min_x', -10.0)
            goal_msg.search_area.min_corner.y = params['search_area'].get('min_y', -10.0)
            goal_msg.search_area.max_corner.x = params['search_area'].get('max_x', 10.0)
            goal_msg.search_area.max_corner.y = params['search_area'].get('max_y', 10.0)

        # Send goal and wait for result
        goal_handle = await self.perception_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error("Perception goal was rejected")
            return False

        result = await goal_handle.get_result_async()

        # Update environment state with perceived information
        if result.result.objects_found:
            self.get_logger().info(f"Found {len(result.result.objects_found)} objects")

        return result.result.success

    async def execute_query_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute query action (information gathering)
        """
        # For now, this just logs the query
        query_text = params.get('query', 'unknown')
        self.get_logger().info(f"Query action: {query_text}")

        # In a real system, this might involve:
        # - Querying a knowledge base
        # - Asking the user for clarification
        # - Checking robot's internal state
        # - Looking up information in the environment

        return True

    async def execute_wait_action(self, params: Dict[str, Any]) -> bool:
        """
        Execute wait action
        """
        duration = params.get('duration', 1.0)  # Default to 1 second
        self.get_logger().info(f"Waiting for {duration} seconds")

        # In a real ROS 2 system, we'd use a timer or similar mechanism
        # For this example, we'll just return True immediately
        # In practice, you'd want to properly wait while keeping the node responsive

        return True
```

## Practical Examples of Cognitive Planning Implementations

### Example 1: Multi-Room Navigation and Object Retrieval

```python
class MultiStepTaskPlanner:
    """
    Example implementation of a complex multi-step task
    """
    def __init__(self, llm_planner: LLMBehaviorPlanner, action_generator: ROS2ActionGenerator):
        self.llm_planner = llm_planner
        self.action_generator = action_generator

    async def execute_fetch_task(self, item_description: str, destination: str) -> bool:
        """
        Execute a task like "Go to the kitchen, find a red cup, and bring it to the living room"
        """
        command = f"Go to the {destination}, find a {item_description}, and bring it to me"

        # Define robot capabilities
        capabilities = {
            "mobility": True,
            "manipulation": True,
            "perception": True
        }

        # Get current environment state (in a real system, this would come from sensors)
        environment_state = {
            "known_rooms": ["kitchen", "living room", "bedroom", "office"],
            "robot_location": "starting_position",
            "known_objects": {
                "kitchen": ["red cup", "blue mug", "plate"],
                "living room": ["sofa", "coffee table", "TV"]
            }
        }

        # Generate plan using LLM
        plan = await self.llm_planner.plan_behavior(
            command,
            capabilities,
            environment_state
        )

        if not plan["action_sequence"]:
            self.action_generator.get_logger().error("No valid action sequence generated")
            return False

        # Execute the plan
        success = await self.action_generator.execute_action_sequence(plan["action_sequence"])

        return success
```

### Example 2: Conditional Task Execution

```python
class ConditionalTaskPlanner:
    """
    Example of handling conditional logic in cognitive planning
    """
    def __init__(self, llm_planner: LLMBehaviorPlanner, action_generator: ROS2ActionGenerator):
        self.llm_planner = llm_planner
        self.action_generator = action_generator
        self.context_store = {}  # Store context between actions

    async def execute_conditional_task(self, command: str) -> bool:
        """
        Execute a task that requires conditional logic
        Example: "If John is in the office, tell him to attend the meeting. Otherwise, send him a message."
        """
        # First, break down the command into conditional parts
        conditional_prompt = f"""
        Analyze the following conditional command and break it into:
        1. The condition to check
        2. The action to take if condition is true
        3. The action to take if condition is false

        Command: "{command}"

        Respond in JSON format:
        {{
            "condition_check": "what to check",
            "if_true_action": "action if condition is met",
            "if_false_action": "action if condition is not met"
        }}
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.llm_planner.model,
                messages=[
                    {"role": "system", "content": "You are a conditional statement analyzer for robotics tasks."},
                    {"role": "user", "content": conditional_prompt}
                ],
                temperature=0.1
            )

            conditional_plan = json.loads(response.choices[0].message.content)

            # First, execute the condition check
            condition_result = await self.check_condition(conditional_plan["condition_check"])

            # Execute appropriate action based on condition
            if condition_result:
                action_to_execute = conditional_plan["if_true_action"]
            else:
                action_to_execute = conditional_plan["if_false_action"]

            # Generate and execute the specific action
            specific_plan = await self.llm_planner.plan_behavior(
                action_to_execute,
                {"mobility": True, "communication": True},  # Capabilities needed
                self.context_store.get("environment", {})
            )

            return await self.action_generator.execute_action_sequence(specific_plan["action_sequence"])

        except Exception as e:
            self.action_generator.get_logger().error(f"Error in conditional task: {str(e)}")
            return False

    async def check_condition(self, condition: str) -> bool:
        """
        Check if a condition is true (in a real system, this would involve perception)
        """
        # In a real system, this would involve:
        # - Using perception to check the environment
        # - Querying a knowledge base
        # - Checking robot sensors

        # For this example, we'll simulate a check
        self.action_generator.get_logger().info(f"Checking condition: {condition}")

        # Simulate a random result for demo purposes
        import random
        return random.choice([True, False])
```

## Configuration Examples for LLM Integration

### LLM Configuration Parameters

```python
# llm_config.py
class LLMConfig:
    # API Configuration
    API_KEY = os.getenv('OPENAI_API_KEY')
    MODEL = os.getenv('LLM_MODEL', 'gpt-4')

    # Planning Parameters
    TEMPERATURE = float(os.getenv('LLM_TEMPERATURE', 0.2))  # Lower for more consistent planning
    MAX_TOKENS = int(os.getenv('LLM_MAX_TOKENS', 1000))

    # Confidence Thresholds
    PLAN_CONFIDENCE_THRESHOLD = float(os.getenv('PLAN_CONFIDENCE_THRESHOLD', 0.7))
    ACTION_CONFIDENCE_THRESHOLD = float(os.getenv('ACTION_CONFIDENCE_THRESHOLD', 0.6))

    # Retry Configuration
    MAX_PLANNING_RETRIES = int(os.getenv('MAX_PLANNING_RETRIES', 3))
    PLANNING_RETRY_DELAY = float(os.getenv('PLANNING_RETRY_DELAY', 1.0))

    # Context Management
    MAX_CONTEXT_HISTORY = int(os.getenv('MAX_CONTEXT_HISTORY', 10))
    CONTEXT_SUMMARIZATION_THRESHOLD = int(os.getenv('CONTEXT_SUMMARIZATION_THRESHOLD', 20))
```

### System Prompt Configuration

```python
# system_prompts.py
SYSTEM_PROMPTS = {
    "behavior_planner": """
    You are an expert robot behavior planner. Your job is to decompose natural language commands
    into executable action sequences for a robot. Consider:

    1. The robot's capabilities and limitations
    2. The current environment and its constraints
    3. The most efficient sequence of actions to achieve the goal
    4. Potential obstacles and how to handle them
    5. Safety considerations

    Always respond in the specified JSON format. Be precise and realistic about what the robot can do.
    """,

    "action_validator": """
    You are an action sequence validator. Your job is to review action sequences generated by
    a behavior planner and identify potential issues. Look for:

    1. Logical sequence of actions
    2. Physical feasibility given robot capabilities
    3. Safety concerns
    4. Missing preconditions
    5. Contradictory actions

    Rate each sequence as HIGH, MEDIUM, or LOW risk and provide specific feedback.
    """,

    "context_summarizer": """
    You are a conversation and task context summarizer. Your job is to condense the history
    of interactions and task executions into a brief summary that preserves important context
    for future planning. Focus on:

    1. Current robot state and location
    2. Recently completed tasks
    3. Known environment information
    4. User preferences or instructions

    Keep the summary concise but informative.
    """
}
```

## Code Snippets for LLM-Based Planning Systems

### Error Handling and Recovery

```python
class ResilientPlanner:
    """
    A planner with built-in error handling and recovery mechanisms
    """
    def __init__(self, llm_planner: LLMBehaviorPlanner, action_generator: ROS2ActionGenerator):
        self.llm_planner = llm_planner
        self.action_generator = action_generator
        self.failure_history = []

    async def execute_with_recovery(self, command: str, context: Dict[str, Any]) -> bool:
        """
        Execute a command with automatic recovery from failures
        """
        # Generate initial plan
        plan = await self.llm_planner.plan_behavior(command,
                                                   context["capabilities"],
                                                   context["environment"])

        if not plan["action_sequence"]:
            return False

        # Execute with failure callback for recovery
        success = await self.action_generator.execute_action_sequence(
            plan["action_sequence"],
            success_callback=self.on_success,
            failure_callback=self.on_failure_and_recovery
        )

        return success

    async def on_failure_and_recovery(self, action_sequence: List[Dict[str, Any]],
                                   failed_step_idx: int, failed_action: Dict[str, Any]):
        """
        Handle failure and attempt recovery
        """
        failed_action_copy = failed_action.copy()
        self.failure_history.append({
            "failed_action": failed_action_copy,
            "step_index": failed_step_idx,
            "timestamp": time.time()
        })

        # If we've failed multiple times on the same action, try alternative
        if self._is_recurring_failure(failed_action):
            await self._try_alternative_approach(action_sequence, failed_step_idx, failed_action)
        else:
            # Try the same action again with modified parameters
            await self._retry_with_modifications(action_sequence, failed_step_idx, failed_action)

    def _is_recurring_failure(self, action: Dict[str, Any]) -> bool:
        """
        Check if this action has failed multiple times recently
        """
        recent_failures = [f for f in self.failure_history
                          if f["failed_action"]["action_type"] == action["action_type"]
                          and time.time() - f["timestamp"] < 300]  # Last 5 minutes

        return len(recent_failures) >= 3

    async def _try_alternative_approach(self, action_sequence: List[Dict[str, Any]],
                                      failed_step_idx: int, failed_action: Dict[str, Any]):
        """
        Try an alternative approach when the same action keeps failing
        """
        # Ask the LLM for an alternative way to achieve the same goal
        alternative_prompt = f"""
        The following action keeps failing. Suggest an alternative approach to achieve the same goal:
        Failed Action: {json.dumps(failed_action)}
        Original Plan: {json.dumps(action_sequence)}

        Respond with alternative actions that could achieve the same outcome.
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.llm_planner.model,
                messages=[
                    {"role": "system", "content": "You are a robotics expert who suggests alternative approaches when actions fail."},
                    {"role": "user", "content": alternative_prompt}
                ]
            )

            # Parse and execute the alternative approach
            # (Implementation would continue from here)

        except Exception as e:
            self.action_generator.get_logger().error(f"Error in alternative approach: {str(e)}")

    async def _retry_with_modifications(self, action_sequence: List[Dict[str, Any]],
                                     failed_step_idx: int, failed_action: Dict[str, Any]):
        """
        Retry the failed action with modifications
        """
        # Modify parameters slightly and try again
        modified_action = failed_action.copy()
        if "precision" in modified_action["parameters"]:
            # If it failed due to precision, try with lower precision
            modified_action["parameters"]["precision"] *= 1.5

        # Attempt to re-execute the modified action
        success = await self.action_generator.execute_single_action(modified_action)

        if not success:
            self.action_generator.get_logger().error("Retry also failed")
```

## Exercises for Cognitive Planning Implementations

### Exercise 1: Basic Multi-Step Planning
Implement a system that can handle simple multi-step commands.

**Requirements**:
- Set up LLM integration for cognitive planning
- Create a planner that can decompose simple multi-step commands
- Execute the resulting action sequences using ROS 2
- Validate that complex commands are properly decomposed

**Steps**:
1. Implement the LLMBehaviorPlanner class
2. Create action sequence validation
3. Connect to ROS 2 action servers
4. Test with multi-step commands like "Go to kitchen and find a cup"

### Exercise 2: Context-Aware Planning
Extend the system to maintain and use context between commands.

**Requirements**:
- Maintain environment state between commands
- Use context to inform planning decisions
- Handle commands that reference previous actions or states
- Implement context summarization for long-running interactions

**Implementation**:
- Create a context manager for environment state
- Modify the LLM planner to use context
- Add context summarization for long histories
- Test with context-dependent commands

### Exercise 3: Error Recovery Planning
Implement sophisticated error handling and recovery mechanisms.

**Requirements**:
- Detect when actions fail during execution
- Generate alternative approaches when failures occur
- Learn from repeated failures to improve future planning
- Provide graceful degradation when plans cannot be executed

**Components**:
- Failure detection and logging
- Alternative approach generation
- Learning from failure patterns
- Graceful degradation mechanisms

## Summary

Cognitive planning with LLMs represents a significant advancement in robotics, enabling robots to understand and execute complex natural language commands through sophisticated reasoning and action decomposition. By combining the reasoning capabilities of LLMs with the precision of ROS 2 action execution, we create systems that can handle the ambiguity and complexity of natural language interaction. This chapter has covered the key components of cognitive planning: LLM integration, natural language decomposition, and ROS 2 action generation, providing the foundation for more advanced autonomous capabilities in the following chapter.