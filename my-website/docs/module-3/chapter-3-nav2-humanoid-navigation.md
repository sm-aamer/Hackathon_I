---
title: Nav2 for Humanoid Navigation
sidebar_position: 3
tags: [nav2, humanoid-navigation, path-planning, bipedal-locomotion, gait-aware-navigation, robotics]
description: Learn about Navigation2 configuration for humanoid robots, focusing on path planning and navigation workflows adapted for bipedal locomotion.
---

# Nav2 for Humanoid Navigation

This chapter covers the configuration and use of Navigation2 (Nav2) for humanoid robots, focusing on path planning and navigation workflows specifically adapted for bipedal locomotion. Unlike traditional wheeled robots, humanoid robots require specialized navigation strategies that account for their unique gait patterns, balance constraints, and mobility characteristics.

## Overview of Nav2 for Humanoid Robots

Navigation2 (Nav2) is ROS 2's state-of-the-art navigation framework, but when applied to humanoid robots, it requires specialized configurations to accommodate the unique challenges of bipedal locomotion. Key considerations include:

Nav2 for humanoid robots builds upon the navigation concepts from standard ROS 2 navigation, which are introduced in [Module 1: The Robotic Nervous System](/docs/module-1/chapter-1-ros2-fundamentals). The humanoid-specific configurations leverage simulation environments that can be created following the principles in [Module 2: The Digital Twin](/docs/module-2/chapter-1-gazebo-basics).

## Cross-Module Connections

- **With Module 1 (ROS 2)**: Nav2 extends the basic navigation concepts from ROS 2 with more sophisticated path planning and control algorithms. The action servers, costmaps, and other navigation components use the ROS 2 foundations you learned in Module 1.
- **With Module 2 (Digital Twin)**: The navigation system can be developed and tested in simulated environments created in Gazebo or Unity before deployment to real humanoid robots, bridging simulation and reality.

- **Gait-aware path planning**: Accounting for humanoid walking patterns and stability
- **Balance-constrained navigation**: Ensuring paths are traversable given balance limitations
- **Footstep planning integration**: Coordinating navigation with footstep placement
- **Dynamic stability**: Maintaining balance during navigation maneuvers

## Key Differences from Wheeled Robot Navigation

### Mobility Constraints
- **Turning mechanisms**: Humanoids turn by stepping rather than rotating in place
- **Step size limitations**: Navigation must respect maximum step distances
- **Balance recovery**: Ability to recover from minor disturbances during movement
- **Terrain negotiation**: Capability to handle stairs, slopes, and uneven terrain differently

### Gait Patterns
- **Double support phase**: Periods where both feet contact the ground
- **Single support phase**: Periods where only one foot contacts the ground
- **Swing leg trajectory**: The path the moving foot takes during steps
- **Zero Moment Point (ZMP) constraints**: Balance requirements during walking

## Nav2 Architecture for Humanoid Robots

### Core Components

#### 1. Global Planner (Humanoid-Aware)
- **Humanoid-optimized path planning**: Algorithms that consider humanoid kinematics
- **Gait pattern integration**: Path planning that accounts for walking patterns
- **Stability-aware optimization**: Ensuring paths are dynamically stable
- **Footstep planning coordination**: Integration with footstep planners

#### 2. Local Planner (Balance-Constrained)
- **Dynamic window approach**: Adapted for humanoid balance constraints
- **Footstep-aware local planning**: Considering upcoming foot placements
- **Balance recovery protocols**: Mechanisms to maintain stability during navigation
- **Reactive obstacle avoidance**: Adjusting gait patterns for obstacles

#### 3. Controller (Gait-Based)
- **Walking pattern generators**: Controllers that generate appropriate gaits
- **Balance maintenance**: Ensuring center of mass remains within stable regions
- **Step timing adaptation**: Adjusting step frequency and duration as needed
- **Transition management**: Smooth transitions between different walking modes

### Humanoid-Specific Plugins

#### Footstep Planner Plugin
```cpp
class HumanoidFootstepPlanner : public nav2_core::FootstepPlanner
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & plugin_name) override;

  std::vector<Footstep> planFootsteps(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::OccupancyGrid & costmap) override;

private:
  double max_step_length_;
  double max_step_width_;
  double foot_separation_;
};
```

#### Balance Constraint Plugin
```cpp
class BalanceConstraintChecker : public nav2_core::ConstraintChecker
{
public:
  bool isTrajectoryValid(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap) override;

  bool isPoseBalanced(
    const geometry_msgs::msg::Pose & pose) override;

private:
  double zmp_margin_;
  double com_height_;
  double max_lean_angle_;
};
```

## Nav2 Configuration for Humanoid Navigation

### Global Planner Configuration

#### Humanoid-Optimized Global Planner Parameters

```yaml
# Global planner parameters for humanoid navigation
global_planner:
  ros__parameters:
    # Basic parameters
    expected_planner_frequency: 20.0
    use_astar: false  # Use RRT* or other humanoid-appropriate algorithms
    allow_unknown: false
    tolerance: 0.5  # Increased tolerance for humanoid path following

    # Humanoid-specific parameters
    min_turn_radius: 0.3  # Minimum turning radius considering step constraints
    max_step_length: 0.6  # Maximum distance between consecutive footsteps
    max_step_width: 0.4   # Maximum lateral step distance
    gait_pattern: "natural_walk"  # Type of walking pattern to assume

    # Balance constraints
    zmp_margin: 0.05      # Zero Moment Point safety margin
    com_height: 0.8       # Center of Mass height for stability
    max_lean_angle: 15.0  # Maximum acceptable lean angle in degrees
```

### Local Planner Configuration

#### DWB (Dynamic Window Approach) for Humanoid Robots

```yaml
# Local planner configuration for humanoid navigation
local_planner:
  ros__parameters:
    # Basic DWB parameters
    desired_linear_vel: 0.3
    max_linear_vel: 0.5
    min_linear_vel: 0.1
    max_angular_vel: 0.5
    min_angular_vel: 0.1

    # Humanoid-specific parameters
    max_translational_accel: 0.5   # Lower acceleration for balance
    max_translational_decel: 1.0   # Higher deceleration for stopping
    max_rotational_vel: 0.3       # Slower turns for stability
    min_rotational_vel: 0.05      # Minimum rotation for fine adjustments

    # Balance-related parameters
    com_safety_margin: 0.1        # Safety margin for center of mass
    step_timing_tolerance: 0.2     # Tolerance for step timing
    balance_recovery_threshold: 0.8 # Threshold for balance recovery mode

    # Footstep planning integration
    footstep_lookahead: 5         # Number of future footsteps to consider
    step_duration: 0.8            # Expected duration of each step
```

### Costmap Configuration

#### Humanoid-Aware Costmaps

```yaml
# Costmap configuration for humanoid navigation
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 20
      height: 20
      resolution: 0.05
      origin_x: -10.0
      origin_y: -10.0

      # Humanoid-specific inflation
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]

      inflation_layer:
        cost_scaling_factor: 1.5    # Adjusted for humanoid safety
        inflation_radius: 0.8       # Larger safety radius for bipedal stability
        inflate_unknown: false

      voxel_layer:
        enabled: true
        voxel_size: 0.2             # Appropriate for humanoid-sized obstacles
        max_obstacle_height: 2.0    # Humanoid can step over small obstacles
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      width: 6
      height: 6
      resolution: 0.05
      origin_x: -3.0
      origin_y: -3.0

      # Humanoid-specific local costmap
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      inflation_layer:
        cost_scaling_factor: 3.0    # Higher scaling for local safety
        inflation_radius: 0.6       # Local inflation for immediate safety
```

## Practical Examples: Humanoid Navigation Scenarios

### Example 1: Basic Humanoid Navigation Setup

```python
#!/usr/bin/env python3
# Basic humanoid navigation setup using Nav2

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import math

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')

        # Initialize the navigator
        self.navigator = BasicNavigator()

        # Wait for Nav2 to activate
        self.navigator.waitUntilNav2Active(localizer='amcl')

        # Set robot-specific parameters
        self.step_length = 0.5  # meters
        self.max_turn_radius = 0.3  # meters
        self.balance_threshold = 0.8  # balance factor

        self.get_logger().info("Humanoid Navigator initialized")

    def create_pose_stamped(self, position_x, position_y, orientation_z, orientation_w):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = orientation_z
        pose.pose.orientation.w = orientation_w
        return pose

    def navigate_with_gait_constraints(self, goal_pose):
        """Navigate to goal with humanoid-specific constraints"""

        # Pre-process the goal to ensure it's reachable with humanoid gait
        adjusted_goal = self.adjust_goal_for_humanoid(goal_pose)

        # Send the goal to Nav2
        self.navigator.goToPose(adjusted_goal)

        # Monitor progress with balance awareness
        while not self.navigator.isTaskComplete():
            # Get feedback on navigation progress
            feedback = self.navigator.getFeedback()

            # Check balance constraints during navigation
            if self.check_balance_constraint():
                # Continue navigation
                pass
            else:
                # Trigger balance recovery
                self.trigger_balance_recovery()

        # Get result
        result = self.navigator.getResult()
        return result

    def adjust_goal_for_humanoid(self, original_goal):
        """Adjust goal pose to be achievable with humanoid gait"""
        adjusted_goal = original_goal

        # Ensure goal is not too close to obstacles considering humanoid size
        # Humanoids need more space to maneuver than wheeled robots
        min_clearance = 0.6  # meters

        # Additional adjustments based on humanoid kinematics
        # Could include checking if the final pose allows for proper foot placement

        return adjusted_goal

    def check_balance_constraint(self):
        """Check if current navigation state maintains balance"""
        # This would typically interface with balance controller
        # For simulation, we'll return True
        return True

    def trigger_balance_recovery(self):
        """Trigger balance recovery mechanism"""
        self.get_logger().warn("Balance recovery triggered!")
        # Implement balance recovery logic
        # This might involve stopping, adjusting stance, etc.

def main(args=None):
    rclpy.init(args=args)

    navigator = HumanoidNavigator()

    # Define a goal
    goal_pose = navigator.create_pose_stamped(
        position_x=2.0,
        position_y=2.0,
        orientation_z=0.0,
        orientation_w=1.0
    )

    try:
        # Navigate to goal with humanoid constraints
        result = navigator.navigate_with_gait_constraints(goal_pose)

        if result:
            navigator.get_logger().info("Goal reached successfully!")
        else:
            navigator.get_logger().info("Failed to reach goal.")

    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Humanoid Footstep Planning Integration

```python
#!/usr/bin/env python3
# Humanoid footstep planning integration with Nav2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np

class HumanoidFootstepPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_footstep_planner')

        # Publishers for visualization
        self.footstep_marker_pub = self.create_publisher(MarkerArray, 'humanoid_footsteps', 10)
        self.footstep_plan_pub = self.create_publisher(MarkerArray, 'footstep_plan', 10)

        # Parameters
        self.foot_separation = 0.3  # Distance between feet when standing
        self.max_step_length = 0.6  # Maximum step forward
        self.max_step_width = 0.4   # Maximum lateral step
        self.step_height = 0.1      # Height of foot during swing phase

        self.get_logger().info("Humanoid Footstep Planner initialized")

    def plan_footsteps(self, start_pose, goal_pose, path):
        """Plan footsteps for humanoid navigation along a path"""

        footsteps = []

        # Convert path poses to footsteps
        for i in range(len(path.poses)):
            # Determine if this should be a left or right footstep
            is_left_foot = (i % 2) == 0  # Alternate feet

            # Create footstep pose
            foot_pose = self.calculate_foot_pose(
                path.poses[i],
                is_left_foot,
                self.foot_separation
            )

            footsteps.append({
                'pose': foot_pose,
                'is_left': is_left_foot,
                'step_number': i
            })

        return footsteps

    def calculate_foot_pose(self, body_pose, is_left_foot, foot_separation):
        """Calculate foot pose based on body pose and which foot"""

        foot_pose = PoseStamped()
        foot_pose.header = body_pose.header

        # Calculate foot offset based on which foot and body orientation
        yaw = self.quaternion_to_yaw(body_pose.pose.orientation)

        # Left foot offset (positive Y) or right foot offset (negative Y)
        if is_left_foot:
            lateral_offset = foot_separation / 2.0
        else:
            lateral_offset = -foot_separation / 2.0

        # Apply rotation to get the actual offset in map frame
        offset_x = -lateral_offset * math.sin(yaw)
        offset_y = lateral_offset * math.cos(yaw)

        foot_pose.pose.position.x = body_pose.pose.position.x + offset_x
        foot_pose.pose.position.y = body_pose.pose.position.y + offset_y
        foot_pose.pose.position.z = 0.0  # On ground

        # Foot orientation matches body orientation
        foot_pose.pose.orientation = body_pose.pose.orientation

        return foot_pose

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def visualize_footsteps(self, footsteps):
        """Publish visualization markers for footsteps"""

        marker_array = MarkerArray()

        for i, footstep in enumerate(footsteps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Position
            marker.pose = footstep['pose'].pose

            # Scale (foot size)
            marker.scale.x = 0.15  # Diameter
            marker.scale.y = 0.15  # Diameter
            marker.scale.z = 0.01  # Thickness

            # Color based on foot type
            if footstep['is_left']:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue for left
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red for right

            marker_array.markers.append(marker)

        self.footstep_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    footstep_planner = HumanoidFootstepPlanner()

    try:
        rclpy.spin(footstep_planner)
    except KeyboardInterrupt:
        pass
    finally:
        footstep_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Balance-Aware Navigation Controller

```python
#!/usr/bin/env python3
# Balance-aware navigation controller for humanoid robots

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Float64
import numpy as np
import math

class HumanoidBalanceController(Node):
    def __init__(self):
        super().__init__('humanoid_balance_controller')

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel_input', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_output', 10)
        self.balance_status_pub = self.create_publisher(Float64, 'balance_status', 10)

        # TF listener for pose information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Balance parameters
        self.com_height = 0.8  # Center of mass height
        self.zmp_margin = 0.05  # ZMP safety margin
        self.balance_threshold = 0.7  # Balance factor threshold
        self.recovery_gain = 2.0  # Recovery gain for balance correction

        # State variables
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.balance_factor = 1.0  # 1.0 = perfectly balanced, 0.0 = fallen

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.balance_control_loop)  # 20Hz

        self.get_logger().info("Humanoid Balance Controller initialized")

    def imu_callback(self, msg):
        """Process IMU data to determine balance state"""

        # Convert quaternion to Euler angles
        orientation = msg.orientation
        self.current_roll, self.current_pitch, self.current_yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Calculate balance factor based on tilt angles
        roll_contribution = abs(self.current_roll) / (math.pi / 6)  # Assuming max stable roll is 30 degrees
        pitch_contribution = abs(self.current_pitch) / (math.pi / 6)  # Same for pitch

        # Balance factor decreases as tilt increases (1.0 = perfect balance, 0.0 = unstable)
        self.balance_factor = max(0.0, 1.0 - (roll_contribution + pitch_contribution) / 2.0)

        # Publish balance status
        balance_msg = Float64()
        balance_msg.data = self.balance_factor
        self.balance_status_pub.publish(balance_msg)

    def cmd_vel_callback(self, msg):
        """Receive velocity commands and apply balance constraints"""

        # Store the desired velocity for the control loop
        self.desired_linear_x = msg.linear.x
        self.desired_angular_z = msg.angular.z

    def balance_control_loop(self):
        """Main balance control loop"""

        # Create output twist message
        output_twist = Twist()

        # Check current balance state
        if self.balance_factor < self.balance_threshold:
            # Robot is becoming unbalanced, apply recovery measures
            self.get_logger().warn(f"Balance factor low: {self.balance_factor:.2f}, applying recovery")

            # Reduce commanded velocities to help recover balance
            output_twist.linear.x = self.desired_linear_x * self.balance_factor * self.recovery_gain
            output_twist.angular.z = self.desired_angular_z * self.balance_factor * self.recovery_gain

            # Optionally add corrective motions to restore balance
            self.apply_balance_correction(output_twist)
        else:
            # Robot is well balanced, use commanded velocities with safety limits
            output_twist.linear.x = self.limit_velocity(
                self.desired_linear_x,
                max_vel=0.5,  # Lower max velocity for humanoid safety
                balance_factor=self.balance_factor
            )

            output_twist.angular.z = self.limit_velocity(
                self.desired_angular_z,
                max_vel=0.3,
                balance_factor=self.balance_factor
            )

        # Publish the controlled velocity command
        self.cmd_vel_pub.publish(output_twist)

    def apply_balance_correction(self, twist_msg):
        """Apply corrective motions to improve balance"""

        # Simple proportional correction based on tilt
        # This would be more sophisticated in a real implementation
        correction_factor = (self.balance_threshold - self.balance_factor) * 5.0

        # Apply linear correction to counteract tilt
        if abs(self.current_roll) > 0.1:  # If tilted significantly in roll
            twist_msg.linear.y += -np.sign(self.current_roll) * correction_factor * 0.1

        if abs(self.current_pitch) > 0.1:  # If tilted significantly in pitch
            twist_msg.linear.x += -np.sign(self.current_pitch) * correction_factor * 0.1

    def limit_velocity(self, desired_vel, max_vel, balance_factor):
        """Limit velocity based on balance factor"""
        limited_vel = max(-max_vel, min(max_vel, desired_vel))

        # Scale by balance factor for safety
        return limited_vel * balance_factor

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    balance_controller = HumanoidBalanceController()

    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration Examples for Humanoid Navigation

### 1. Complete Nav2 Configuration for Humanoid

```yaml
# main_nav2_humanoid_config.yaml

amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_delay: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific BT configuration
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_costmap_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.1
    min_theta_velocity_threshold: 0.1
    # Humanoid-specific controllers
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["HumanoidMpcController"]  # Humanoid-specific controller

    HumanoidMpcController:
      plugin: "nav2_mppi_controller::Controller"
      time_steps: 26
      dt: 0.1
      horizon: 2.5
      # Humanoid-specific MPC parameters
      discretization_method: "midpoint"
      iteration_count: 3
      enable_antialiasing: false
      model_predictive_path_integral:
        transform_tolerance: 0.2
        frequency: 20.0
        xy_goal_tolerance: 0.25
        trans_stopped_velocity: 0.25
        theta_goal_tolerance: 0.4
        rot_stopped_velocity: 0.5
        n_workers: 3
        regularization_lambda: 0.0
        motion_model: "Ackermann"
        weight:
          goal_distance: 1.0
          path_distance: 2.0
          goal_angle: 0.5
          path_angle: 0.2
          obstacle: 5.0
          dynamic_obstacle: 10.0
          constraint: 100.0
          goal_angle_current_pose: 0.0
        noise:
          model_type: "Mixed"
          theta: 0.5
          vx_std: 0.2
          vy_std: 0.0
          omega_std: 0.3
          vtheta_mean: 0.0
          vtheta_std: 0.1
        ackermann:
          min_turning_r: 0.3  # Humanoid-specific turning radius
          max_steering_angle: 0.5
          max_longitudinal_acc: 0.5  # Reduced for balance
          min_longitudinal_acc: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific parameters
      robot_radius: 0.4  # Larger radius for humanoid safety
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.6
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: "/pointcloud"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.1
          raytrace_max_range: 3.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: False
      robot_radius: 0.4  # Humanoid-specific radius
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 1.5
        inflation_radius: 0.8
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: "/pointcloud"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_max_range: 10.0
          obstacle_min_range: 0.1
          raytrace_max_range: 10.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      # Use a planner appropriate for humanoid navigation
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: false
      # Humanoid-specific parameters
      max_step_length: 0.6  # Maximum distance between waypoints
      max_deviation: 0.3    # Maximum deviation from straight line

smoother_server:
  ros__parameters:
    use_sim_time: False
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True
      # Humanoid-specific smoothing parameters
      smooth_weight: 0.05   # Less aggressive smoothing for precise footstep planning
      curvature_weight: 0.2 # Balance between smoothness and curvature
      smooth_scale: 1.0

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      # Humanoid-specific spin parameters
      spin_dist: 0.5  # Reduced spin distance for humanoid
      time_allowance: 10
    backup:
      plugin: "nav2_behaviors::BackUp"
      # Humanoid-specific backup parameters
      backup_dist: 0.3  # Shorter backup distance
      backup_speed: 0.1
      time_allowance: 10
    wait:
      plugin: "nav2_behaviors::Wait"
      time_allowance: 10
```

## Troubleshooting Humanoid Navigation Issues

### Common Navigation Problems and Solutions

#### 1. Unstable Walking During Navigation
- **Problem**: Robot becomes unstable or falls during navigation
- **Causes**:
  - Velocity commands too aggressive for balance
  - Poor ZMP (Zero Moment Point) management
  - Inadequate feedback control
- **Solutions**:
  - Reduce maximum linear and angular velocities
  - Implement balance feedback control
  - Use appropriate gait patterns for commanded speeds
  - Add safety margins to navigation parameters

#### 2. Footstep Planning Failures
- **Problem**: Robot cannot find valid footsteps to reach goal
- **Causes**:
  - Overly constrained footstep planning
  - Obstacles too close to navigable path
  - Inaccurate robot dimensions in configuration
- **Solutions**:
  - Adjust step length and width constraints
  - Improve costmap resolution for footstep planning
  - Implement step recovery behaviors
  - Use more flexible footstep planning algorithms

#### 3. Localization Drift in Long-Term Navigation
- **Problem**: Robot loses accurate position estimate over time
- **Causes**:
  - Accumulated odometry errors from walking
  - Limited sensing capabilities during bipedal locomotion
  - Uneven terrain affecting sensor readings
- **Solutions**:
  - Enhance localization with multiple sensors (IMU, cameras, LIDAR)
  - Implement loop closure detection
  - Use visual-inertial odometry for better accuracy
  - Regular map-based relocalization

#### 4. Turning Difficulties
- **Problem**: Robot struggles to execute turns properly
- **Causes**:
  - Turning radius too tight for humanoid kinematics
  - Inadequate balance control during turns
  - Poor coordination between steps during rotation
- **Solutions**:
  - Increase minimum turning radius in configuration
  - Implement specialized turning gaits
  - Use step-and-turn sequences rather than in-place rotation
  - Add balance control during turning maneuvers

### Performance Optimization Strategies

#### 1. Computation Time Reduction
- **Efficient path planning**: Use hierarchical planning with coarse global and fine local planners
- **Parallel processing**: Execute perception, planning, and control in parallel threads
- **Predictive planning**: Precompute potential paths based on likely goals

#### 2. Balance Maintenance
- **Proactive balance control**: Anticipate balance challenges and adjust gait accordingly
- **Adaptive step timing**: Modify step duration based on terrain and obstacle requirements
- **Recovery strategies**: Implement graceful recovery from near-fall situations

#### 3. Energy Efficiency
- **Optimal gait selection**: Choose energy-efficient walking patterns
- **Smooth transitions**: Minimize energy waste during direction changes
- **Terrain adaptation**: Adjust gait for different surface types

## Exercises for Humanoid Navigation

### Exercise 1: Basic Humanoid Navigation Setup
Configure Nav2 for basic humanoid navigation in a simple environment.

**Requirements:**
- Set up Nav2 with humanoid-appropriate parameters
- Configure costmaps for humanoid dimensions
- Test basic navigation in a simple environment
- Verify balance-aware velocity control

**Steps:**
1. Install Nav2 packages for humanoid navigation
2. Configure robot-specific parameters (dimensions, step size, etc.)
3. Set up localization (AMCL) for humanoid robot
4. Test navigation in gazebo simulation
5. Evaluate performance metrics (success rate, time to goal, balance maintenance)

### Exercise 2: Gait-Aware Path Planning
Implement and test gait-aware path planning for humanoid robots.

**Requirements:**
- Configure path planner with humanoid kinematic constraints
- Implement footstep planning integration
- Test path feasibility with actual walking patterns
- Evaluate path optimality considering gait requirements

**Implementation:**
- Modify global planner to consider humanoid step constraints
- Integrate with footstep planner
- Validate paths with kinematic simulation
- Compare with traditional wheeled robot planning

### Exercise 3: Balance-Constrained Local Navigation
Implement local navigation with balance constraints for humanoid robots.

**Requirements:**
- Develop balance-aware local planner
- Integrate IMU feedback for real-time balance adjustment
- Test obstacle avoidance while maintaining stability
- Evaluate balance recovery behaviors

**Components:**
- Balance monitoring system
- Velocity limiting based on stability
- Reactive obstacle avoidance with balance preservation
- Recovery behaviors for near-instability situations

### Exercise 4: Terrain-Aware Navigation
Extend navigation system to handle varied terrain types.

**Requirements:**
- Adapt navigation for stairs, slopes, and uneven terrain
- Implement terrain classification and gait selection
- Test navigation on challenging terrain
- Evaluate robustness and safety

**Features:**
- Terrain type detection
- Gait pattern selection based on terrain
- Adjusted navigation parameters per terrain type
- Safe transition between different terrains

### Exercise 5: Multi-Humanoid Coordination
Implement coordinated navigation for multiple humanoid robots.

**Requirements:**
- Extend navigation system for multi-robot scenarios
- Implement collision avoidance between humanoids
- Test formation navigation
- Evaluate scalability and communication requirements

**Challenges:**
- Inter-humanoid collision avoidance
- Communication-limited coordination
- Formation maintenance during navigation
- Scalability to larger groups

## Summary

Nav2 for humanoid navigation requires specialized configurations that account for the unique challenges of bipedal locomotion. Unlike wheeled robots, humanoids must consider balance constraints, gait patterns, and footstep planning during navigation. Successful implementation requires careful tuning of parameters, integration with balance control systems, and specialized planning algorithms that respect the kinematic and dynamic constraints of bipedal walking. With proper configuration, Nav2 can enable capable and safe navigation for humanoid robots in various environments.

## Next Steps

Explore related topics in Module 3:

- [Chapter 1: Isaac Sim Essentials](./chapter-1-isaac-sim-essentials.md) - Simulation fundamentals for training
- [Chapter 2: Isaac ROS Integration](./chapter-2-isaac-ros-integration.md) - Perception and integration concepts

Or explore other modules:

- [Module 1: The Robotic Nervous System (ROS 2)](/docs/module-1/chapter-1-ros2-fundamentals) - Fundamentals of ROS 2
- [Module 2: The Digital Twin (Gazebo & Unity)](/docs/module-2/chapter-1-gazebo-basics) - Simulation and interaction concepts