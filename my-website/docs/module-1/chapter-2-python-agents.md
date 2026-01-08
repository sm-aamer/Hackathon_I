---
id: chapter-2-python-agents
title: Chapter 2 - Python Agents + rclpy Integration
sidebar_label: Python Agents + rclpy
---

# Chapter 2: Python Agents + rclpy Integration

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides the standard interface for Python programs to communicate with ROS 2. It allows Python programs to create ROS 2 nodes, publish and subscribe to topics, make service calls, provide services, and create action clients and servers.

### Installing rclpy

rclpy comes as part of the ROS 2 Python client libraries. To use it, you need to have ROS 2 installed on your system and source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
```

## Connecting AI Agents to ROS Controllers

In this chapter, we'll explore how to connect Python-based AI agents to ROS controllers, bridging artificial intelligence with physical robot control. This integration allows AI algorithms to interact with robotic systems in real-time, enabling intelligent behaviors to be executed on real or simulated robots.

### AI Agent Architecture

A typical AI agent connected to ROS 2 would have the following components:

1. **Perception Layer**: Subscribes to sensor data topics
2. **Decision Making**: Processes sensor data and makes decisions
3. **Action Layer**: Publishes commands to robot controllers
4. **Communication Layer**: Manages ROS 2 node interfaces

## Writing Publishers and Subscribers with rclpy

### Minimal Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class AIPublisher(Node):
    def __init__(self):
        super().__init__('ai_publisher')

        # Create publisher for motor commands
        self.motor_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/motor_commands',
            10
        )

        # Timer to periodically send commands
        self.timer = self.create_timer(0.1, self.publish_motor_commands)

        # Simulated AI decision making
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]

    def publish_motor_commands(self):
        """Publish motor commands based on AI decision"""
        msg = Float64MultiArray()
        msg.data = self.joint_angles

        self.motor_cmd_publisher.publish(msg)
        self.get_logger().info(f'Published motor commands: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    ai_publisher = AIPublisher()

    try:
        rclpy.spin(ai_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ai_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Minimal Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class AISubscriber(Node):
    def __init__(self):
        super().__init__('ai_subscriber')

        # Create subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10
        )

        self.latest_sensor_data = None

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.latest_sensor_data = {
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'effort': list(msg.effort)
        }

        self.get_logger().info(f'Received sensor data: {len(msg.position)} joints')

def main(args=None):
    rclpy.init(args=args)

    ai_subscriber = AISubscriber()

    try:
        rclpy.spin(ai_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        ai_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating AI Algorithms

### Example: PID Controller as AI Agent

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class PIDControllerAgent(Node):
    def __init__(self):
        super().__init__('pid_controller_agent')

        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Float64MultiArray, '/motor_commands', 10)
        self.sensor_subscriber = self.create_subscription(
            JointState, '/joint_states', self.sensor_callback, 10
        )

        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        self.dt = 0.01  # Control loop time step

        # PID state
        self.error_prev = 0.0
        self.integral = 0.0

        # Target positions
        self.target_positions = [1.0, 0.5, -0.5, 0.0]

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.current_positions = [0.0, 0.0, 0.0, 0.0]

    def sensor_callback(self, msg):
        """Update current positions from sensor data"""
        self.current_positions = list(msg.position)

    def control_loop(self):
        """Main control loop implementing PID algorithm"""
        # Calculate errors
        errors = [target - current for target, current in
                  zip(self.target_positions, self.current_positions)]

        # Proportional term
        p_term = [self.kp * error for error in errors]

        # Integral term
        self.integral = [i + e * self.dt for i, e in zip(self.integral, errors)]
        i_term = [self.ki * i for i in self.integral]

        # Derivative term
        derivative = [(e - ep) / self.dt for e, ep in zip(errors, self.error_prev)]
        self.error_prev = errors
        d_term = [self.kd * d for d in derivative]

        # Calculate control output
        control_output = [p + i + d for p, i, d in zip(p_term, i_term, d_term)]

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = control_output
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    controller_agent = PIDControllerAgent()

    try:
        rclpy.spin(controller_agent)
    except KeyboardInterrupt:
        pass
    finally:
        controller_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples

### Example 1: Simple Reflex Agent

A reflex agent that responds directly to sensor inputs:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ReflexAgent(Node):
    def __init__(self):
        super().__init__('reflex_agent')

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

    def scan_callback(self, msg):
        """Simple obstacle avoidance reflex"""
        min_distance = min(msg.ranges)

        cmd = Twist()
        if min_distance > 1.0:  # Safe distance
            cmd.linear.x = 0.5   # Move forward
            cmd.angular.z = 0.0  # Go straight
        else:  # Obstacle detected
            cmd.linear.x = 0.0   # Stop
            cmd.angular.z = 0.5  # Turn

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    reflex_agent = ReflexAgent()

    try:
        rclpy.spin(reflex_agent)
    except KeyboardInterrupt:
        pass
    finally:
        reflex_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Goal-Seeking Agent

An agent that navigates toward a goal while avoiding obstacles:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
import math

class GoalSeekingAgent(Node):
    def __init__(self):
        super().__init__('goal_seeking_agent')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Goal position
        self.goal_x = 5.0
        self.goal_y = 5.0

        # Robot position (simplified)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        self.safe_distance = 0.5

    def scan_callback(self, msg):
        """Process laser scan and navigate toward goal"""
        # Simplified navigation logic
        cmd = Twist()

        # Calculate angle to goal
        angle_to_goal = math.atan2(
            self.goal_y - self.robot_y,
            self.goal_x - self.robot_x
        )

        # Check for obstacles in front
        front_scan = msg.ranges[len(msg.ranges)//2]  # Approximate front

        if front_scan < self.safe_distance:
            # Obstacle in front - turn away
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        else:
            # Move toward goal
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angle_to_goal * 0.5  # Simple proportional control

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    goal_agent = GoalSeekingAgent()

    try:
        rclpy.spin(goal_agent)
    except KeyboardInterrupt:
        pass
    finally:
        goal_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. Implement a neural network-based agent that learns to control a simple robot
2. Create an agent that coordinates multiple robot behaviors (navigation, manipulation, etc.)
3. Develop a reinforcement learning agent that learns optimal control policies for a humanoid robot