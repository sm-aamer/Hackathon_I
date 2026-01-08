---
title: Isaac Sim Essentials
sidebar_position: 1
tags: [isaac-sim, simulation, robotics, photorealistic-rendering, physics-simulation, synthetic-data]
description: Learn about NVIDIA Isaac Sim fundamentals including photorealistic rendering, physics simulation, and synthetic data generation for humanoid robotics.
---

# Isaac Sim Essentials

This chapter covers the fundamentals of NVIDIA Isaac Sim, a powerful simulation environment for robotics development. Isaac Sim provides photorealistic rendering, accurate physics simulation, and synthetic data generation capabilities that are essential for training AI models for humanoid robots.

## Overview of Isaac Sim

NVIDIA Isaac Sim is a comprehensive robotics simulation environment built on NVIDIA Omniverse. It provides:

This simulation environment builds upon the ROS 2 fundamentals covered in [Module 1: The Robotic Nervous System](/module-1/chapter-1-ros2-fundamentals), allowing you to simulate the ROS 2 nodes, topics, and services you learned about. Additionally, Isaac Sim can complement the digital twin concepts from [Module 2: The Digital Twin](/module-2/chapter-1-gazebo-basics) by providing photorealistic rendering capabilities that enhance the realism of your simulated environments.

## Cross-Module Connections

- **With Module 1 (ROS 2)**: Isaac Sim can interface with ROS 2 nodes to provide realistic simulation environments for your ROS 2-based robots. The simulation can publish and subscribe to ROS 2 topics just like a real robot would.
- **With Module 2 (Digital Twin)**: While Gazebo provides physics simulation, Isaac Sim enhances this with photorealistic rendering and advanced sensor simulation that can generate synthetic training data for AI models.

- **Photorealistic rendering**: High-fidelity visual simulation using RTX ray tracing
- **Accurate physics simulation**: Realistic physics interactions with PhysX engine
- **Synthetic data generation**: Tools for creating labeled training data
- **Robot simulation**: Support for complex robotic systems and sensors
- **AI training environment**: Reinforcement learning and imitation learning capabilities

## Installation and Setup

To get started with Isaac Sim, you'll need:

1. NVIDIA GPU with RTX capabilities (recommended: RTX 3080 or higher)
2. CUDA-compatible drivers
3. Isaac Sim installation from NVIDIA Omniverse
4. Compatible ROS/ROS2 environment for robotics workflows

## Key Concepts

Isaac Sim operates on several core concepts that are essential for effective simulation:

### USD (Universal Scene Description)
Isaac Sim uses NVIDIA's Universal Scene Description (USD) as its core scene representation format. USD allows for:
- Hierarchical scene composition
- Layer-based editing and collaboration
- Extensible schemas for robotics content
- Efficient streaming and rendering

### Robot Definition Files
Robots in Isaac Sim are defined using:
- URDF (Unified Robot Description Format) for basic kinematic structure
- MJCF (MuJoCo XML) for dynamic simulation
- Isaac-specific extensions for advanced sensor and actuator models

### Simulation Stages
Isaac Sim organizes simulation content into different stages:
- **World Stage**: Contains the environment and static objects
- **Robot Stage**: Contains the robot definition and initial configuration
- **Task Stage**: Defines the specific task or scenario to be simulated

## Photorealistic Rendering Capabilities

Isaac Sim's rendering pipeline provides industry-leading photorealistic visualization through NVIDIA's RTX technology:

### RTX Ray Tracing
- **Global illumination simulation**: Advanced light transport for realistic indirect lighting
- **Accurate reflections and refractions**: True-to-life mirror-like surfaces and transparent materials
- **Realistic lighting models**: Physically-based lighting that matches real-world behavior
- **Physically-based materials**: Advanced shader models with realistic surface properties

### Lighting Systems
- **Dynamic lighting**: Real-time adjustment of light sources and intensities
- **Environmental lighting**: HDR environment maps for realistic scene illumination
- **Area lights**: Soft shadows and realistic light distribution
- **Volumetric effects**: Fog, smoke, and atmospheric scattering simulation

### Synthetic Data Generation
- **RGB image generation**: Photorealistic color images with accurate color reproduction
- **Depth maps**: Precise distance measurements for 3D reconstruction
- **Semantic segmentation masks**: Pixel-level classification for AI training
- **Instance segmentation masks**: Object-specific labeling for detection tasks
- **Normal maps**: Surface orientation data for geometric analysis
- **Material property maps**: Information about surface characteristics
- **Optical flow**: Motion vector data for temporal analysis
- **Surface normals**: Geometric orientation for 3D understanding

### Rendering Quality Settings
- **Multi-sample anti-aliasing (MSAA)**: Smooth edges and reduced aliasing
- **Temporal denoising**: Clean images with reduced noise in ray tracing
- **Adaptive sampling**: Optimized rendering performance without quality loss
- **Resolution scaling**: Flexible output resolution for different applications

## Isaac Sim Physics Simulation

Isaac Sim provides advanced physics simulation capabilities through its integration with NVIDIA PhysX, enabling realistic interactions between robots, objects, and environments:

### PhysX Integration
- **Accurate collision detection**: Advanced algorithms for detecting complex shape interactions using GPU-accelerated broad-phase and CPU-based narrow-phase collision detection
- **Realistic contact response**: Physically accurate friction, bounce, and contact forces with support for complex contact manifolds
- **Multi-body dynamics simulation**: Complex articulated systems with multiple joints, constraints, and degrees of freedom
- **Soft body simulation capabilities**: Deformable objects, cloth simulation, and fluid dynamics
- **Rigid body simulation**: High-fidelity simulation of rigid objects with mass, inertia, and collision properties

### Advanced Physics Features
- **Contact graphs**: Efficient representation of contact relationships for complex multi-object interactions
- **Joint constraints**: Revolute, prismatic, fixed, spherical, and universal joint types with drive capabilities
- **Articulated body dynamics**: Specialized solvers for robotic systems with complex kinematic chains
- **Custom force fields**: User-defined forces and torques for specialized simulation scenarios

### Simulation Parameters
- **Gravity configuration**: Adjustable gravity vectors for different environments (Earth, Moon, Mars, zero-G)
- **Time step control**: Adaptive and fixed time stepping with support for sub-stepping for stability
- **Solver parameters**: Iterative solvers with configurable position and velocity error correction
- **Contact material properties**: Customizable friction coefficients, restitution, and surface properties
- **Damping parameters**: Linear and angular damping for realistic motion decay
- **Sleep thresholds**: Energy-based sleeping for performance optimization of static objects

### Performance Optimization
- **Level of detail (LOD)**: Automatic simplification of complex geometries during simulation
- **Contact filtering**: Efficient culling of unnecessary collision checks
- **Broad-phase optimization**: Spatial partitioning for efficient collision detection in large scenes
- **Multi-threading**: Parallel processing of physics calculations across CPU cores

## Synthetic Data Generation Workflows

Isaac Sim excels at generating synthetic datasets for AI model training, providing realistic data at scale:

### Data Generation Pipeline
- **Scene randomization**: Systematic variation of lighting, textures, and object positions
- **Sensor simulation**: Accurate modeling of real-world sensors including noise and distortion
- **Annotation generation**: Automatic ground truth labeling for various computer vision tasks
- **Batch processing**: Automated generation of large-scale datasets

### Workflow Components
- **Asset libraries**: Extensive collections of objects, materials, and environments
- **Randomization scripts**: Programs that systematically vary scene parameters
- **Data collection tools**: Interfaces for capturing and storing simulation data
- **Quality assurance**: Validation tools to ensure data quality and consistency

### Common Data Types
- **Image datasets**: RGB, depth, and multi-spectral image collections
- **Sensor fusion data**: Combined data from multiple sensor types
- **Temporal sequences**: Time-series data for motion analysis
- **Multi-view datasets**: Synchronized data from multiple camera viewpoints

### Data Quality Assurance
- **Domain randomization**: Systematic variation to improve model generalization
- **Physical accuracy**: Ensuring synthetic data matches real-world physics
- **Label accuracy**: Precise ground truth annotations for supervised learning
- **Statistical validation**: Verification that synthetic data matches real-world distributions

### Integration with AI Training
- **Direct pipeline integration**: Seamless connection to popular ML frameworks
- **Format compatibility**: Output in standard formats (COCO, KITTI, etc.)
- **Data augmentation**: Built-in tools for expanding dataset diversity
- **Performance metrics**: Tools for evaluating synthetic vs. real data similarity

## Practical Examples: Isaac Sim Environment Setup

### Example 1: Basic Scene Creation

Let's create a simple scene with a robot and environment:

```python
# Basic Isaac Sim scene setup using the Python API
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive

# Initialize the simulation world
my_world = World(stage_units_in_meters=1.0)

# Create a ground plane
create_primitive(
    prim_path="/World/GroundPlane",
    primitive_props={"size": 10.0},
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Props/Grid/default_plane_prim.usda"
)

# Add a robot to the scene (using a sample robot)
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka_panda_alt1.usd",
    prim_path="/World/Robot"
)

# Add an object for the robot to interact with
create_primitive(
    prim_path="/World/Cube",
    primitive_type="Cube",
    scale=[0.1, 0.1, 0.1],
    position=[0.5, 0.0, 0.5]
)

# Reset the world to apply changes
my_world.reset()

# Run simulation steps
for i in range(100):
    my_world.step(render=True)

# Cleanup
my_world.clear()
```

### Example 2: Advanced Environment with Custom Assets

For more complex environments, you can load custom USD files:

```python
# Advanced environment setup with custom assets
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Initialize the simulation world
world = World(stage_units_in_meters=1.0)

# Get path to Isaac Sim assets
assets_root_path = get_assets_root_path()

# Add a complex environment scene
if assets_root_path:
    house_scene_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
    add_reference_to_stage(
        usd_path=house_scene_path,
        prim_path="/World/Room"
    )

# Add a humanoid robot
robot_asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
add_reference_to_stage(
    usd_path=robot_asset_path,
    prim_path="/World/Humanoid"
)

# Configure physics properties
world.reset()
```

### Example 3: Dynamic Scene Configuration

Creating scenes with programmatically adjustable parameters:

```python
# Dynamic scene configuration with adjustable parameters
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import set_targets
from pxr import UsdLux

def create_dynamic_scene(robot_type="franka", env_type="simple", lighting_conditions="neutral"):
    """Create a scene with configurable parameters"""

    # Initialize world
    world = World(stage_units_in_meters=1.0)

    # Add environment based on type
    if env_type == "simple":
        # Add simple room
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/Room"
        )
    elif env_type == "warehouse":
        # Add warehouse environment
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            prim_path="/World/Warehouse"
        )

    # Add robot based on type
    if robot_type == "franka":
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka_panda_alt1.usd",
            prim_path="/World/Robot"
        )
    elif robot_type == "humanoid":
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Humanoid/humanoid_instanceable.usd",
            prim_path="/World/Robot"
        )

    # Configure lighting based on conditions
    if lighting_conditions == "neutral":
        # Add dome light for neutral lighting
        dome_light = UsdLux.DomeLight.Define(world.scene.stage, "/World/Light/DomeLight")
        dome_light.CreateIntensityAttr(1000)
        dome_light.CreateColorAttr((0.8, 0.8, 0.9))
    elif lighting_conditions == "bright":
        # Add brighter lighting
        dome_light = UsdLux.DomeLight.Define(world.scene.stage, "/World/Light/DomeLight")
        dome_light.CreateIntensityAttr(2000)
        dome_light.CreateColorAttr((1.0, 1.0, 1.0))

    # Apply configuration
    world.reset()
    return world

# Create a custom scene
custom_world = create_dynamic_scene(robot_type="humanoid", env_type="warehouse", lighting_conditions="bright")
```

### Example 4: Configurable Physics Properties

Setting up physics with customizable parameters:

```python
# Configurable physics environment
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.core.utils.stage import set_physics_material
from omni.physx.scripts.physicsUtils import *

# Create world with custom physics parameters
world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0/60.0,  # Physics timestep
    rendering_dt=1.0/60.0  # Rendering timestep
)

# Create a ground plane with custom physics material
ground = create_primitive(
    prim_path="/World/GroundPlane",
    primitive_type="Plane",
    scale=[10, 10, 1],
    position=[0, 0, 0]
)

# Set physics material properties
set_physics_material(
    prim_path="/World/GroundPlane",
    static_friction=0.5,
    dynamic_friction=0.5,
    restitution=0.1  # Bounciness
)

# Create objects with different materials
cube1 = create_primitive(
    prim_path="/World/Cube1",
    primitive_type="Cube",
    scale=[0.2, 0.2, 0.2],
    position=[0.5, 0.0, 1.0],
    orientation=[0, 0, 0, 1],
    color=[0.8, 0.2, 0.2]  # Red cube
)

# Apply different physics properties to the cube
set_physics_material(
    prim_path="/World/Cube1",
    static_friction=0.1,
    dynamic_friction=0.1,
    restitution=0.8  # Very bouncy
)

# Reset world to apply physics properties
world.reset()
```

## Configuration Examples

### Isaac Sim Configuration Files

Isaac Sim uses various configuration formats to define simulation parameters. Here are examples of different configuration approaches:

#### 1. JSON Configuration for Scene Parameters

```json
{
  "scene": {
    "gravity": [0, 0, -9.81],
    "time_step": 0.00833,
    "solver_type": "TGS",
    "collision_margin": 0.001,
    "default_physics_material": {
      "static_friction": 0.5,
      "dynamic_friction": 0.5,
      "restitution": 0.1
    }
  },
  "rendering": {
    "resolution": [1920, 1080],
    "fps": 60,
    "ray_tracing": true,
    "multi_sampling": 4,
    "max_surface_bounces": 8,
    "max_volume_bounces": 4,
    "max_diffuse_bounces": 4,
    "max_refraction_bounces": 8
  },
  "sensors": {
    "camera": {
      "resolution": [640, 480],
      "fov": 60,
      "clipping_range": [0.1, 100.0]
    },
    "lidar": {
      "rotation_rate": 10,
      "samples_per_scan": 1080,
      "number_of_layers": 16,
      "points_per_second": 1000000
    }
  },
  "environment": {
    "asset_path": "omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Room/simple_room.usd",
    "lighting": {
      "intensity": 3000,
      "color": [1.0, 1.0, 1.0],
      "type": "dome"
    }
  }
}
```

#### 2. Python Configuration Script

```python
# Configuration script for Isaac Sim scene
import carb
import omni
from omni.isaac.core.utils.stage import set_stage_units
from omni.isaac.core import World
from omni.isaac.core.utils.physics import set_physics_dt
from omni.isaac.core.utils.prims import set_prim_translation
from omni.isaac.core.utils.semantics import add_semantic_annotation

def configure_simulation():
    """Configure the simulation with specific parameters"""

    # Set stage units
    set_stage_units(1.0)  # meters

    # Configure physics parameters
    set_physics_dt(
        physics_dt=1.0/60.0,      # Physics timestep
        rendering_dt=1.0/60.0,    # Rendering timestep
        stage_prim=None
    )

    # Get physics scene
    scene = omni.usd.get_context().get_stage().GetPrimAtPath("/World/PhysicsScene")

    # Configure solver parameters
    if scene.IsValid():
        # Set solver type
        scene.GetAttribute("physxScene:useStabilization").Set(True)
        scene.GetAttribute("physxScene:solverType").Set("TGS")
        scene.GetAttribute("physxScene:enableCCD").Set(True)
        scene.GetAttribute("physxScene:enableAdaptiveForce").Set(True)

    # Configure gravity
    world_interface = omni.physics.get_world_interface()
    if world_interface:
        world_interface.set_gravity(0, 0, -9.81)

def configure_robot(robot_prim_path, initial_position):
    """Configure robot-specific parameters"""
    # Set initial position
    set_prim_translation(robot_prim_path, initial_position)

    # Add semantic annotations if needed
    add_semantic_annotation(robot_prim_path, "class", "robot")

def configure_sensors(robot_prim_path):
    """Configure sensors attached to the robot"""
    # Example: Configure a camera sensor
    camera_config = {
        "resolution": [1280, 720],
        "fov": 90.0,
        "clipping_range": [0.1, 100.0]
    }

    # Example: Configure LIDAR sensor
    lidar_config = {
        "rotation_rate": 10,
        "samples_per_scan": 1080,
        "number_of_layers": 16
    }

    return camera_config, lidar_config

# Apply configuration
configure_simulation()
```

#### 3. USD Configuration for Robot Definition

```usd
# Example USD file for robot configuration
# robot_config.usd

# Define the robot prim
def Xform "Robot" (
    prepend apiSchemas = ["IsaacArticulatedRobot"]
)
{
    # Robot properties
    uniform token physics:articulationSolverType = "TGS"
    float3 physics:jointFriction = (0.1, 0.1, 0.1)

    # Base link
    def Xform "base_link" (
        prepend apiSchemas = ["IsaacRigidBody"]
    )
    {
        # Base link properties
        float3 physics:mass = (10.0, 10.0, 10.0)
        float3 physics:centerOfMass = (0, 0, 0)

        # Collision properties
        def Capsule "collision" (
            prepend apiSchemas = ["IsaacCollisionAPI"]
        )
        {
            float radius = 0.2
            float height = 0.5
        }

        # Visual properties
        def Capsule "visual" (
            prepend apiSchemas = ["IsaacMeshAPI"]
        )
        {
            float radius = 0.2
            float height = 0.5
        }
    }

    # Joint definition
    def PhysicsJoint "joint1" (
        prepend apiSchemas = ["IsaacJointAPI"]
    )
    {
        # Joint properties
        float physics:jointFriction = 0.1
        float physics:jointDamping = 0.1
        float3 physics:jointAxis = (0, 0, 1)
    }
}
```

#### 4. Environment Configuration Template

```yaml
# Environment configuration for Isaac Sim
environment:
  name: "warehouse_simulation"
  description: "Large warehouse environment for mobile robot navigation"

  assets:
    main_scene: "omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    objects:
      - "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Kiva/shelf.usd"
      - "omniverse://localhost/NVIDIA/Assets/Isaac/Props/Blocks/block_20cm.usd"

  lighting:
    type: "dome"
    intensity: 3000
    color: [0.9, 0.9, 1.0]  # Slightly blue-tinted white
    texture: "omniverse://localhost/NVIDIA/Assets/Isaac/Environments/Simple_Warehouse/Background/dark_1_8k.hdr"

  physics:
    gravity: [0.0, 0.0, -9.81]
    solver_type: "TGS"
    substeps: 1
    fixed_timestep: 0.016667  # 60 Hz

  rendering:
    resolution: [1920, 1080]
    max_ray_length: 100.0
    enable_denoising: true
    msaa_samples: 4

  sensors:
    default_camera:
      resolution: [640, 480]
      fov: 60.0
      clipping_range: [0.1, 100.0]

    lidar_3d:
      rotation_rate: 10
      samples_per_scan: 1080
      number_of_layers: 16
      points_per_second: 1000000
      clipping_range: [0.1, 25.0]
```

## Troubleshooting Common Issues

### Rendering Performance
- **Reduce scene complexity**: Simplify geometry or use lower-poly models if experiencing low frame rates
- **Adjust rendering quality**: Lower ray tracing quality or disable advanced effects (denoising, global illumination) based on GPU capabilities
- **Use level-of-detail (LOD)**: Implement LOD models for distant objects to reduce rendering load
- **Resolution scaling**: Temporarily reduce viewport resolution during simulation development
- **Texture compression**: Use compressed texture formats to reduce GPU memory usage
- **Occlusion culling**: Hide objects that are not in the camera's field of view

### Advanced Rendering Issues
- **Ray tracing artifacts**: Increase ray tracing samples or adjust denoising parameters
- **Lighting inconsistencies**: Verify that light intensities and color temperatures are physically plausible
- **Reflection/refraction problems**: Check material properties and ensure proper UV mapping
- **Shadow quality**: Adjust shadow map resolution and bias settings for better shadow quality
- **Overexposure**: Fine-tune exposure settings and HDR tone mapping parameters

### Physics Stability
- **Jittery physics**: Reduce physics timestep (e.g., to 1/120s or 1/240s) or adjust solver iterations
- **Object penetration**: Verify mass properties, collision shapes, and increase contact stiffness/damping
- **Joint instability**: Check joint limits, drive parameters, and ensure proper mass distribution
- **Explosive simulations**: Reduce time step, check for extreme mass ratios, or adjust solver parameters
- **Floating objects**: Verify that objects have proper mass and collision properties assigned

### Memory and Performance Optimization
- **GPU memory monitoring**: Use NVIDIA Nsight Graphics or System Management to monitor VRAM usage
- **Asset streaming**: Implement level streaming for large environments to load/unload sections dynamically
- **Texture streaming**: Use lower-resolution textures for distant objects or implement texture streaming
- **Simulation batching**: Process multiple simulation scenarios in batches to optimize resource usage
- **Cache optimization**: Enable USD stage caching for frequently accessed assets

### Asset Loading Problems
- **Missing assets**: Verify asset paths are accessible and properly formatted (OmniGraph/USD paths)
- **USD stage errors**: Check USD file validity using USD viewer tools or validator scripts
- **Material issues**: Ensure materials are properly authored and compatible with Isaac Sim's renderer
- **Animation problems**: Verify animation data is properly embedded in USD files
- **Scale mismatches**: Check that all assets use consistent units (typically meters for Isaac Sim)

### Network and Connection Issues
- **Nucleus server connectivity**: Verify Isaac Sim can connect to Omniverse Nucleus server for assets
- **Local asset paths**: Ensure local USD files are referenced with proper "file://" protocol
- **Firewall restrictions**: Check if network security is blocking Omniverse connections
- **Bandwidth limitations**: For cloud-based simulation, ensure sufficient network bandwidth

### Common Error Messages and Solutions

| Error Message | Cause | Solution |
|---------------|-------|----------|
| "Failed to initialize PhysX" | GPU driver or PhysX compatibility issue | Update GPU drivers and verify CUDA compatibility |
| "Out of memory" | Insufficient GPU or system RAM | Reduce scene complexity or increase system resources |
| "Stage failed to load" | Invalid USD file or inaccessible path | Validate USD file and check path permissions |
| "NaN detected in physics" | Invalid physical properties or calculations | Check for zero masses, infinite values, or invalid transforms |
| "Renderer initialization failed" | Graphics driver or hardware compatibility | Update graphics drivers and verify RTX hardware |

### Debugging Tips
- **Enable verbose logging**: Use Isaac Sim's logging features to diagnose issues
- **Visual debugging**: Use Isaac Sim's debug visualization tools to inspect physics properties
- **Step-by-step execution**: Run simulation at reduced speed to observe problematic behaviors
- **Minimal reproducible case**: Isolate issues by creating simplified test scenarios
- **Check system requirements**: Verify that hardware meets Isaac Sim's minimum requirements

## Code Snippets for Isaac Sim Configuration

This section provides practical code examples for configuring Isaac Sim environments, robots, and sensors.

### 1. Basic Isaac Sim Initialization

```python
# Initialize Isaac Sim environment
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_primitive

# Create a world instance with specific parameters
world = World(
    stage_units_in_meters=1.0,      # Set stage units to meters
    physics_dt=1.0/60.0,           # Physics timestep (60 Hz)
    rendering_dt=1.0/60.0,         # Rendering timestep (60 Hz)
    backend="torch",               # Backend for physics calculations
    device="cuda"                  # Device for computation
)

# Add a ground plane to the scene
create_primitive(
    prim_path="/World/GroundPlane",
    primitive_type="Plane",
    scale=[10.0, 10.0, 1.0],
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0, 1.0]
)

# Reset the world to apply changes
world.reset()
```

### 2. Robot Loading and Configuration

```python
# Load and configure a robot in Isaac Sim
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_active_viewport_camera_settings

# Define robot configuration
robot_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka_panda_alt1.usd"
robot_prim_path = "/World/Robot"

# Add robot to the stage
add_reference_to_stage(
    usd_path=robot_usd_path,
    prim_path=robot_prim_path
)

# Create robot object for control
robot = Robot(
    prim_path=robot_prim_path,
    name="franka_robot",
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0, 1.0]
)

# Reset world after adding robot
world.reset()

# Example: Move robot to initial position
initial_positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # Position and orientation
robot.set_world_pose(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
```

### 3. Sensor Configuration

```python
# Configure various sensors in Isaac Sim
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import _range_sensor
import numpy as np

# Create a camera sensor
camera = Camera(
    prim_path="/World/Robot/base_link/Camera",
    name="camera_sensor",
    position=[0.2, 0.0, 0.1],
    frequency=30,
    resolution=(640, 480)
)

# Configure camera properties
camera.set_focal_length(24.0)
camera.set_horizontal_aperture(20.955)
camera.set_vertical_aperture(15.2908)

# Create a LIDAR sensor
lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
lidar_path = "/World/Robot/base_link/Lidar"

# Add LIDAR to the robot
lidar_config = {
    "rotation_frequency": 10,
    "samples_per_scan": 1080,
    "number_of_layers": 16,
    "points_per_second": 1000000,
    "horizontal_fov": 360,
    "vertical_fov": 30,
    "range_distance": 25.0
}

# Initialize sensor data structures
camera.initialize()
lidar_interface.initialize()

# Example: Capture sensor data
def capture_sensor_data():
    # Capture RGB image
    rgb_image = camera.get_rgb()

    # Capture depth data
    depth_data = camera.get_depth()

    # Capture LIDAR data
    lidar_data = lidar_interface.get_point_cloud_data(lidar_path)

    return rgb_image, depth_data, lidar_data
```

### 4. Physics Configuration

```python
# Configure physics properties for objects
from omni.isaac.core.utils.stage import set_physics_properties
from omni.isaac.core.utils.prims import set_collision_enabled, set_mass
from pxr import PhysxSchema, UsdPhysics

# Set global physics properties
set_physics_properties(
    stage=world.scene.stage,
    gravity=9.81,
    solver_type="TGS"  # TGS or PGS solver
)

# Configure collision and mass for an object
object_prim_path = "/World/Object"
set_collision_enabled(object_prim_path, True)
set_mass(object_prim_path, mass=1.0)

# Access and modify PhysX-specific properties
prim = world.scene.stage.GetPrimAtPath(object_prim_path)
if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
    collision_api = PhysxSchema.PhysxCollisionAPI(prim)
    collision_api.GetRestOffsetAttr().Set(0.01)  # Rest offset
    collision_api.GetContactOffsetAttr().Set(0.02)  # Contact offset
```

### 5. Animation and Control Loop

```python
# Main simulation loop with robot control
import numpy as np

def run_simulation_loop(num_steps=1000):
    """Run the simulation with robot control"""

    # Initialize controllers if needed
    world.reset()

    for step in range(num_steps):
        # Perform control calculations
        # Example: Simple joint position control
        if step < 100:  # First 100 steps
            # Move to initial configuration
            joint_positions = [0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.0]
        else:
            # Move to different configuration
            joint_positions = [0.5, -1.5, 0.5, -2.5, 0.0, 1.0, 0.0]

        # Apply joint positions (this would be specific to your robot)
        # robot.get_articulation_controller().apply_position_cmd(joint_positions)

        # Step the world
        world.step(render=True)

        # Capture sensor data periodically
        if step % 30 == 0:  # Every 30 steps
            rgb_img, depth_data, lidar_data = capture_sensor_data()
            print(f"Captured sensor data at step {step}")

    print("Simulation completed")

# Run the simulation
run_simulation_loop()
```

### 6. Environment Randomization

```python
# Randomize environment for synthetic data generation
import random

def randomize_environment():
    """Apply randomization to improve synthetic data diversity"""

    # Randomize lighting
    light_intensity_range = (500, 2000)
    new_intensity = random.uniform(*light_intensity_range)

    # Randomize object positions within bounds
    x_bounds = (-2.0, 2.0)
    y_bounds = (-2.0, 2.0)
    z_bounds = (0.1, 1.0)

    random_x = random.uniform(*x_bounds)
    random_y = random.uniform(*y_bounds)
    random_z = random.uniform(*z_bounds)

    # Apply new position to an object
    # set_prim_translation("/World/Object", [random_x, random_y, random_z])

    # Randomize colors/textures
    random_color = [
        random.uniform(0.2, 1.0),
        random.uniform(0.2, 1.0),
        random.uniform(0.2, 1.0)
    ]

    # Apply random color (implementation depends on material system)
    # apply_color_to_object("/World/Object", random_color)

    print(f"Environment randomized: intensity={new_intensity:.2f}, position=({random_x:.2f}, {random_y:.2f}, {random_z:.2f})")

# Example: Randomize environment before each episode
for episode in range(10):
    randomize_environment()
    run_simulation_loop()
```

### 7. Data Collection and Storage

```python
# Collect and save simulation data
import json
import os
from datetime import datetime

def collect_and_save_data(rgb_image, depth_data, lidar_data, joint_states, save_dir="data/"):
    """Collect and save simulation data"""

    # Create timestamped directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    episode_dir = os.path.join(save_dir, f"episode_{timestamp}")
    os.makedirs(episode_dir, exist_ok=True)

    # Save RGB image
    # Image.fromarray(rgb_image).save(os.path.join(episode_dir, "rgb.png"))

    # Save depth data
    # np.save(os.path.join(episode_dir, "depth.npy"), depth_data)

    # Save LIDAR data
    # np.save(os.path.join(episode_dir, "lidar.npy"), lidar_data)

    # Save joint states
    joint_data = {
        "positions": joint_states.positions.tolist() if hasattr(joint_states, 'positions') else [],
        "velocities": joint_states.velocities.tolist() if hasattr(joint_states, 'velocities') else [],
        "efforts": joint_states.efforts.tolist() if hasattr(joint_states, 'efforts') else []
    }

    with open(os.path.join(episode_dir, "joint_states.json"), 'w') as f:
        json.dump(joint_data, f, indent=2)

    # Save metadata
    metadata = {
        "timestamp": timestamp,
        "episode": episode,
        "scene_config": "default_warehouse_scene",
        "robot_config": "franka_panda",
        "sensor_config": {
            "camera_resolution": [640, 480],
            "lidar_config": lidar_config
        }
    }

    with open(os.path.join(episode_dir, "metadata.json"), 'w') as f:
        json.dump(metadata, f, indent=2)

    print(f"Data saved to {episode_dir}")

# Example usage in simulation loop
# rgb_img, depth_data, lidar_data = capture_sensor_data()
# collect_and_save_data(rgb_img, depth_data, lidar_data, robot.get_joint_state())
```

## Exercises

### Exercise 1: Basic Scene Creation
Create a simple scene with a ground plane and a basic robot model.

**Requirements:**
- Create a ground plane using `create_primitive()` function
- Add a robot from the Isaac Sim asset library
- Position the robot appropriately above the ground plane
- Verify the scene loads correctly and the robot is stable

**Solution Outline:**
```python
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize world
world = World(stage_units_in_meters=1.0)

# Create ground plane
create_primitive(
    prim_path="/World/GroundPlane",
    primitive_type="Plane",
    scale=[10.0, 10.0, 1.0],
    position=[0.0, 0.0, 0.0]
)

# Add robot (example with Franka Panda)
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/Robots/Franka/franka_panda_alt1.usd",
    prim_path="/World/Robot"
)

# Reset the world
world.reset()
```

### Exercise 2: Lighting Configuration
Experiment with different lighting setups to achieve photorealistic rendering.

**Requirements:**
- Add a dome light with adjustable intensity
- Create at least two different lighting scenarios (e.g., indoor vs outdoor)
- Compare the rendering quality between different lighting configurations
- Document the effect of different lighting on synthetic data generation

**Advanced Challenge:**
- Implement dynamic lighting that changes during simulation
- Add area lights for more realistic shadows

### Exercise 3: Physics Validation
Test the physics simulation by dropping objects and observing realistic collisions.

**Requirements:**
- Create objects with different physical properties (mass, friction, restitution)
- Drop objects from different heights and observe their behavior
- Verify that collision detection works properly
- Test joint constraints and verify they behave as expected

**Validation Checklist:**
- Objects fall at approximately 9.81 m/s²
- Objects bounce realistically based on restitution values
- Friction causes objects to slow down appropriately
- Collisions conserve momentum properly

### Exercise 4: Sensor Integration
Add multiple sensors to a robot and verify they produce valid data.

**Requirements:**
- Add a camera sensor to the robot
- Add a LIDAR sensor to the robot
- Configure both sensors with appropriate parameters
- Capture and visualize data from both sensors
- Verify that sensor data is physically plausible

**Expected Output:**
- RGB images showing the robot's perspective
- Depth maps with accurate distance measurements
- Point cloud data from LIDAR sensor
- All data properly synchronized

### Exercise 5: Environment Randomization
Implement a randomization scheme that varies object positions and lighting conditions.

**Requirements:**
- Create a function that randomly places objects in the environment
- Randomize lighting properties (intensity, color temperature)
- Randomize material properties of objects
- Verify that the randomization produces diverse but physically plausible scenarios

**Bonus Challenge:**
- Implement domain randomization to improve AI model generalization
- Track statistics about the generated variations

### Exercise 6: Advanced Physics Tuning
Fine-tune physics parameters for specific simulation requirements.

**Requirements:**
- Adjust solver parameters for improved stability
- Modify contact properties to achieve desired object interactions
- Tune joint properties for realistic robot movement
- Benchmark performance vs. accuracy trade-offs

### Exercise 7: Synthetic Data Pipeline
Create a complete pipeline for generating synthetic training data.

**Requirements:**
- Combine scene randomization with sensor capture
- Implement data annotation and labeling
- Save captured data in standard formats
- Verify data quality and consistency across captures

**Deliverables:**
- Working data generation pipeline
- Sample dataset with annotations
- Quality metrics report

## Summary

Isaac Sim provides a powerful platform for robotics simulation with photorealistic rendering and accurate physics. Understanding these fundamentals will prepare you for more advanced simulation tasks and synthetic data generation for AI training.

## Next Steps

Continue with the next topics in Module 3:

- [Chapter 2: Isaac ROS Integration](./chapter-2-isaac-ros-integration.md) - Learn how to integrate Isaac with ROS for accelerated perception
- [Chapter 3: Nav2 for Humanoid Navigation](./chapter-3-nav2-humanoid-navigation.md) - Explore navigation solutions for humanoid robots

Or explore other modules:

- [Module 1: The Robotic Nervous System (ROS 2)](/module-1/chapter-1-ros2-fundamentals) - Fundamentals of ROS 2
- [Module 2: The Digital Twin (Gazebo & Unity)](/module-2/chapter-1-gazebo-basics) - Simulation and interaction concepts