---
id: chapter-1-gazebo-basics
sidebar_label: Gazebo Simulation Basics
title: Chapter 1 - Gazebo Simulation Basics
---

# Chapter 1: Gazebo Simulation Basics

## Introduction to Gazebo for Robotics Simulation

Gazebo is a powerful robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It plays a crucial role in the development and testing of robotics applications, especially for humanoid robotics where physical testing can be expensive and time-consuming.

Gazebo provides:
- Realistic physics simulation using ODE, Bullet, Simbody, and DART engines
- High-fidelity rendering with OGRE engine
- Convenient programmatic interfaces via Gazebo API
- Extensive sensor models for perception simulation
- Integration with ROS/ROS 2 for robotics development

## Physics Engines in Gazebo

Gazebo supports multiple physics engines including ODE, Bullet, Simbody, and DART. Each engine has its strengths and is suitable for different types of simulation scenarios. Understanding these engines is fundamental to creating accurate simulations.

### ODE (Open Dynamics Engine)

ODE is the default physics engine in Gazebo. It's well-tested and suitable for most robotics simulation tasks. ODE excels at:
- Stable simulation of articulated bodies
- Efficient collision detection
- Good performance with moderate complexity scenes

Example configuration for ODE in a world file:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Bullet Physics

Bullet offers more advanced features and is particularly good for complex contact scenarios. It provides:
- Better handling of complex contacts and friction
- More sophisticated constraint solving
- Improved stability in certain scenarios

### Simbody Physics

Simbody is a multibody dynamics engine that provides:
- Advanced constraint handling
- Good performance for biomechanical simulations
- Excellent numerical stability

### DART Physics

DART (Dynamic Animation and Robotics Toolkit) offers:
- Advanced contact handling
- Humanoid and biomechanical simulation capabilities
- Good for complex articulated systems

## Gravity Models and Configuration

In Gazebo, gravity is applied globally to the simulation environment. The default gravity vector is (0, 0, -9.8), simulating Earth's gravitational acceleration.

### Configuring Gravity

Gravity can be customized in the world file:

```xml
<sdf version='1.6'>
  <world name='default'>
    <gravity>0 0 -9.8</gravity>

    <!-- Optional: Change gravity for different planetary bodies -->
    <!-- Moon gravity: 0 0 -1.62 -->
    <!-- Mars gravity: 0 0 -3.71 -->

    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Collision Detection Systems

Collision detection in Gazebo is handled through collision shapes defined for each object. Accurate collision models are essential for realistic physics interactions.

### Types of Collisions

- Static collisions: Between static objects
- Dynamic collisions: Between moving objects
- Contact joints: When objects come into contact

### Collision Shapes

Gazebo supports several primitive collision shapes:

- Box: `<box><size>1.0 1.0 1.0</size></box>`
- Sphere: `<sphere><radius>0.5</radius></sphere>`
- Cylinder: `<cylinder><radius>0.5</radius><length>1.0</length></cylinder>`
- Capsule: `<capsule><radius>0.1</radius><length>0.8</length></capsule>`
- Mesh: `<mesh><uri>model://my_model/meshes/part.dae</uri></mesh>`

### Collision Properties

Each collision element can specify properties like:
- Surface friction coefficients
- Bounce restitution
- Contact parameters for stability

Example collision configuration:
```xml
<collision name='collision'>
  <geometry>
    <box>
      <size>0.5 0.5 0.5</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
  </surface>
</collision>
```

## Practical Examples

### Setting up a Basic Physics Simulation

Here's a complete example of setting up a simple physics simulation with a box falling onto a ground plane:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="basic_physics">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Configuration File Examples for Gazebo Physics

### World Configuration File (my_world.world)

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_simulation">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Example humanoid base link -->
    <model name="humanoid_base">
      <pose>0 0 1.0 0 0 0</pose>
      <static>false</static>
      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <iyy>1.0</iyy>
            <izz>1.0</izz>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyz>0.0</iyz>
          </inertia>
        </inertial>

        <collision name="collision">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.3 0.8</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.3 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Troubleshooting Tips for Common Gazebo Physics Issues

### Issue: Objects falling through surfaces
- Check that static models have `<static>true</static>` property
- Verify collision geometry is properly defined
- Ensure adequate physics update rates

### Issue: Unstable or jittery simulations
- Reduce max_step_size in physics configuration
- Adjust solver parameters
- Check mass and inertia properties

### Issue: Slow simulation performance
- Simplify collision meshes where possible
- Reduce physics update rate if accuracy allows
- Limit the number of complex contact points

## Exercises for Students

1. Create a simple world with a sphere falling onto a plane surface
2. Experiment with different gravity values to simulate different planets
3. Add multiple objects with different physical properties and observe their interactions
4. Modify the collision properties to create bouncy or sticky objects