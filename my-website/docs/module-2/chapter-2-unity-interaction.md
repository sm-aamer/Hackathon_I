---
id: chapter-2-unity-interaction
sidebar_label: Unity for Human-Robot Interaction
title: Chapter 2 - Unity for Human-Robot Interaction
---

# Chapter 2: Unity for Human-Robot Interaction

## Introduction to Unity for Robotics

Unity is a powerful game development engine that has found increasing applications in robotics simulation and human-robot interaction design. Its high-fidelity rendering capabilities and rich ecosystem of tools make it an excellent choice for creating immersive human-robot interaction scenarios.

Unity offers several advantages for robotics:
- High-fidelity rendering with physically-based materials
- Flexible lighting systems with real-time shadows
- Powerful animation and physics systems
- Extensive asset ecosystem through the Unity Asset Store
- Cross-platform deployment capabilities
- Strong community and documentation support

## Setting Up Unity for Robotics

Setting up Unity for robotics applications requires specific packages and configurations to handle robot models and their interactions effectively.

### Unity Version Requirements

For robotics applications, Unity 2021.3 LTS or later is recommended for:
- Long-term support and stability
- Improved physics engine performance
- Enhanced rendering capabilities
- Better scripting performance

### Unity Robotics Package

The Unity Robotics package provides essential tools for robotics simulation, including ROS integration and sensor simulation capabilities.

#### Installing Unity Robotics Package

1. Open Unity Hub and create a new project
2. In the Unity Editor, go to Window → Package Manager
3. Click the "+" icon and select "Add package from git URL..."
4. Add the Unity Robotics packages:
   - `com.unity.robotics.ros-tcp-connector`
   - `com.unity.robotics.urdf-importer`
   - `com.unity.robotics.robotics-from-unity`

#### ROS Integration

The ROS TCP Connector enables communication between Unity and ROS/ROS 2:
```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("joint_states");
    }

    void Update()
    {
        // Send joint states to ROS
        JointStateMsg jointState = new JointStateMsg();
        // Populate joint state message
        ros.Publish("joint_states", jointState);
    }
}
```

## Environment Creation in Unity

Creating realistic environments for humanoid robots in Unity involves several key components:

### Terrain and Ground Surfaces

Creating realistic terrain that humanoid robots can navigate and interact with.

#### Creating Basic Terrain

1. In Unity Editor, GameObject → 3D Object → Terrain
2. Use the terrain tools to sculpt the landscape
3. Apply textures for different ground types
4. Add trees and vegetation using the terrain tools

#### Terrain Settings for Robotics

For robotics applications, consider:
- Appropriate scale (1 Unity unit = 1 meter for realism)
- Collision detection for navigation
- Texture resolution suitable for perception tasks
- LOD (Level of Detail) settings for performance

### Obstacles and Interactive Elements

Designing objects that robots can perceive and interact with in the environment.

#### Creating Interactive Objects

```csharp
using UnityEngine;

public class InteractiveObject : MonoBehaviour
{
    public bool isGraspable = true;
    public string objectType = "object";

    void Start()
    {
        // Add colliders for physics interaction
        if (!GetComponent<Collider>())
            gameObject.AddComponent<BoxCollider>();

        // Add rigidbody if object should be movable
        if (isGraspable && !GetComponent<Rigidbody>())
            gameObject.AddComponent<Rigidbody>();
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Robot"))
        {
            // Handle robot interaction
            OnRobotInteraction(other.gameObject);
        }
    }

    void OnRobotInteraction(GameObject robot)
    {
        // Implement interaction logic
        Debug.Log($"Robot interacted with {gameObject.name}");
    }
}
```

#### Physics Materials

For realistic physics interactions:
- Create PhysicMaterial assets for different surface properties
- Configure friction and bounce values appropriately
- Apply materials to colliders for accurate simulation

### NavMesh for Robot Navigation

Setting up navigation meshes for pathfinding:

1. Select walkable surfaces
2. Go to Navigation window (Window → AI → Navigation)
3. Bake the NavMesh
4. Use NavMeshAgent for pathfinding in humanoid robots

## Rendering Systems and Materials

Unity's rendering pipeline offers advanced capabilities for creating photorealistic robot models and environments.

### Universal Render Pipeline (URP)

For robotics applications, URP (Universal Render Pipeline) is often recommended:
- Better performance for real-time simulation
- Lower system requirements
- Good visual quality for perception tasks

#### Setting up URP

1. Window → Package Manager → Universal RP
2. Create a new URP Asset: Assets → Create → Rendering → Universal Render Pipeline → Pipeline Asset
3. Assign the asset in Project Settings → Graphics → Scriptable Render Pipeline Settings

### Shader Selection for Robotics

Choosing appropriate shaders for different robot components and environmental elements.

#### Metallic vs Standard Shaders

For metallic robot components:
- Use Metallic workflow with appropriate metallic maps
- Configure smoothness for different surface finishes

For organic/environmental elements:
- Use Standard shader with specular workflow
- Adjust albedo and normal maps for realistic appearance

#### Custom Shaders for Sensors

For sensor simulation, custom shaders can visualize:
- LiDAR point clouds
- Depth camera data
- Field of view visualization

Example sensor visualization shader:
```hlsl
Shader "Robotics/SensorVisualization"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _SensorColor ("Sensor Color", Color) = (1, 0, 0, 1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float4 _SensorColor;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);
                return lerp(col, _SensorColor, 0.3);
            }
            ENDCG
        }
    }
}
```

### Lighting Systems

Setting up realistic lighting that affects how robots perceive their environment.

#### Light Types for Robotics

- Directional lights: Simulate sun or overhead lighting
- Point lights: Represent localized light sources
- Spot lights: Model flashlight or headlight effects
- Area lights: For soft, realistic lighting (requires URP/LWRP)

#### Lighting Considerations for Perception

For perception simulation:
- Use realistic light intensities (lux values)
- Consider dynamic lighting conditions
- Account for shadows affecting sensor data
- Include ambient lighting for realistic rendering

## Human-Robot Interaction Elements

Creating interfaces and elements that facilitate natural interaction between humans and robots in the simulation environment.

### User Interface Design

#### Canvas Setup for HRI

```csharp
using UnityEngine;
using UnityEngine.UI;

public class HumanRobotInterface : MonoBehaviour
{
    public Canvas canvas;
    public Button[] robotControls;
    public Text statusText;

    void Start()
    {
        // Initialize UI elements
        foreach(Button btn in robotControls)
        {
            btn.onClick.AddListener(() => OnRobotControl(btn.name));
        }
    }

    void OnRobotControl(string command)
    {
        // Send command to robot
        Debug.Log($"Sending command: {command}");
    }
}
```

### Gesture Recognition

For simulating gesture-based interaction:

1. Use Unity's Input System for gesture capture
2. Implement gesture recognition algorithms
3. Provide visual feedback for recognized gestures

### Voice Command Simulation

For voice interaction simulation:
- Use Unity's audio system
- Implement keyword recognition
- Provide visual feedback for command acknowledgment

## Practical Examples

### Example 1: Creating a Basic Humanoid Robot Scene

1. Import robot model (URDF or FBX format)
2. Set up basic lighting
3. Create a simple environment
4. Add basic controls

### Example 2: Unity Scene with Humanoid Robot Navigation

```csharp
using UnityEngine;
using UnityEngine.AI;

public class HumanoidNavigator : MonoBehaviour
{
    public Transform target;
    public NavMeshAgent agent;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        if(target != null)
        {
            agent.SetDestination(target.position);
        }
    }
}
```

### Example 3: Sensor Visualization in Unity

Creating a script to visualize sensor data:

```csharp
using UnityEngine;

public class SensorVisualizer : MonoBehaviour
{
    public float detectionRadius = 5.0f;
    public LayerMask detectionLayers;

    void OnDrawGizmosSelected()
    {
        // Draw sensor detection range in editor
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, detectionRadius);
    }

    void FixedUpdate()
    {
        // Detect objects in sensor range
        Collider[] hits = Physics.OverlapSphere(transform.position, detectionRadius, detectionLayers);

        foreach(Collider hit in hits)
        {
            if(hit.transform != transform) // Don't detect self
            {
                Debug.Log($"Sensor detected: {hit.name}");
            }
        }
    }
}
```

## Integration with Robotics Frameworks

### Unity-Rosbridge Integration

For ROS integration:
- Use unity-rosbridge for communication
- Set up publishers/subscribers for robot data
- Implement service clients for robot commands

### Perception Pipeline

Setting up perception simulation:
- Configure cameras for RGB and depth data
- Set up LiDAR simulation using raycasting
- Process sensor data for perception algorithms

## Exercises for Students

1. Create a Unity scene with a simple humanoid robot navigating a maze
2. Implement a basic human-robot interface with buttons and status indicators
3. Set up a perception system with cameras and process the image data
4. Create an interactive environment where the robot responds to human gestures
5. Implement a basic LiDAR simulation system and visualize the point cloud data