---
id: chapter-3-sensor-simulation
sidebar_label: Sensor Simulation
title: Chapter 3 - Sensor Simulation
---

# Chapter 3: Sensor Simulation

## Introduction to Sensor Simulation in Robotics

Sensor simulation is a critical aspect of robotics development that allows for testing perception algorithms without the need for physical hardware. This chapter covers the simulation of various sensor types including LiDAR, depth cameras, and IMUs within Gazebo and Unity environments.

Sensor simulation offers several advantages:
- Cost-effective testing of perception algorithms
- Controlled experimental conditions
- Reproducible scenarios
- Safe development of navigation and control systems
- Accelerated development cycles

## LiDAR Simulation

Light Detection and Ranging (LiDAR) sensors are crucial for mapping and navigation in robotics. Simulating LiDAR data accurately is essential for developing and testing SLAM algorithms.

### LiDAR Sensor Characteristics

LiDAR sensors produce 3D point clouds by measuring distances to objects using laser pulses. Key characteristics include:
- Range: Typical ranges from 1m to 100m+ depending on sensor
- Angular resolution: Horizontal and vertical resolution (e.g., 0.1° to 0.5°)
- Field of view: Horizontal (e.g., 360°) and vertical (e.g., -30° to +10°)
- Scan frequency: Typically 5-20 Hz
- Accuracy: Millimeter-level precision for close objects

### Gazebo LiDAR Simulation

Gazebo provides plugins for simulating various LiDAR sensors with realistic noise models and characteristics.

#### Ray Sensor Plugin

The Gazebo Ray plugin simulates LiDAR sensors using ray tracing:

```xml
<sensor name="lidar_sensor" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <visualize>true</visualize>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

#### Multi-Ray LiDAR Simulation

For 3D LiDAR sensors with vertical beams:

```xml
<sensor name="3d_lidar_sensor" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <visualize>true</visualize>
  <ray>
    <scan>
      <horizontal>
        <samples>1440</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
        <max_angle>0.1745</max_angle>    <!-- 10 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>120.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

#### Noise Modeling for LiDAR

Adding realistic noise to LiDAR measurements:

```xml
<sensor name="lidar_with_noise" type="ray">
  <!-- Previous configuration -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
  </noise>
</sensor>
```

### Unity LiDAR Simulation

Unity offers different approaches for simulating LiDAR data, often using raycasting techniques.

#### LiDAR Simulation Script

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LiDARSimulator : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public int horizontalSamples = 720;
    public int verticalSamples = 1;
    public float minAngle = -Mathf.PI;
    public float maxAngle = Mathf.PI;
    public float verticalMinAngle = 0;
    public float verticalMaxAngle = 0;
    public float maxRange = 30.0f;
    public LayerMask detectionLayers = -1;

    [Header("Output")]
    public bool visualizeRays = true;
    public float pointSize = 0.05f;

    private List<Vector3> pointCloud;
    private float horizontalStep;
    private float verticalStep;

    void Start()
    {
        pointCloud = new List<Vector3>();
        horizontalStep = (maxAngle - minAngle) / horizontalSamples;
        verticalStep = (verticalMaxAngle - verticalMinAngle) / verticalSamples;
    }

    void FixedUpdate()
    {
        pointCloud.Clear();

        for (int v = 0; v < verticalSamples; v++)
        {
            float vAngle = verticalMinAngle + (v * verticalStep);

            for (int h = 0; h < horizontalSamples; h++)
            {
                float hAngle = minAngle + (h * horizontalStep);

                Vector3 direction = GetDirection(hAngle, vAngle);
                RaycastHit hit;

                if (Physics.Raycast(transform.position, direction, out hit, maxRange, detectionLayers))
                {
                    pointCloud.Add(hit.point);

                    if (visualizeRays)
                    {
                        Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                    }
                }
                else
                {
                    // Add point at maximum range if no hit
                    Vector3 farPoint = transform.position + direction * maxRange;
                    pointCloud.Add(farPoint);

                    if (visualizeRays)
                    {
                        Debug.DrawRay(transform.position, direction * maxRange, Color.blue);
                    }
                }
            }
        }
    }

    Vector3 GetDirection(float hAngle, float vAngle)
    {
        // Convert spherical coordinates to Cartesian
        float x = Mathf.Cos(vAngle) * Mathf.Sin(hAngle);
        float y = Mathf.Sin(vAngle);
        float z = Mathf.Cos(vAngle) * Mathf.Cos(hAngle);

        return new Vector3(x, y, z).normalized;
    }

    void OnRenderObject()
    {
        GL.Begin(GL.POINTS);
        GL.Color(Color.red);

        foreach (Vector3 point in pointCloud)
        {
            GL.Vertex(point);
        }

        GL.End();
    }

    // Method to get point cloud data
    public Vector3[] GetPointCloud()
    {
        return pointCloud.ToArray();
    }
}
```

#### Performance Optimization for LiDAR Simulation

For real-time performance:
- Reduce the number of raycasts during gameplay
- Use spatial partitioning for collision detection
- Implement level-of-detail for distant objects
- Cache raycast results when possible

## Depth Camera Simulation

Depth cameras provide 3D spatial information that is vital for perception and navigation tasks in robotics.

### RGB-D Camera Models

Simulating both color and depth information for robotic vision applications.

#### Depth Camera Configuration in Gazebo

```xml
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <baseline>0.2</baseline>
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>depth_camera</cameraName>
    <imageTopicName>/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>depth_camera_frame</frameName>
    <pointCloudCutoff>0.5</pointCloudCutoff>
    <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    <CxPrime>0</CxPrime>
    <Cx>320.5</Cx>
    <Cy>240.5</Cy>
    <focalLength>320</focalLength>
    <hackBaseline>0.07</hackBaseline>
  </plugin>
</sensor>
```

#### Unity Depth Camera Simulation

```csharp
using UnityEngine;

public class DepthCameraSimulator : MonoBehaviour
{
    [Header("Camera Configuration")]
    public int width = 640;
    public int height = 480;
    public float nearClip = 0.1f;
    public float farClip = 10.0f;
    public float fieldOfView = 60.0f;

    [Header("Output")]
    public RenderTexture depthTexture;
    public Shader depthShader;

    private Camera cam;
    private Texture2D depthReadback;

    void Start()
    {
        cam = GetComponent<Camera>();
        SetupDepthCamera();
    }

    void SetupDepthCamera()
    {
        cam.fieldOfView = fieldOfView;
        cam.nearClipPlane = nearClip;
        cam.farClipPlane = farClip;

        // Create depth texture
        depthTexture = new RenderTexture(width, height, 24);
        depthTexture.format = RenderTextureFormat.Depth;
        depthTexture.Create();

        cam.targetTexture = depthTexture;
    }

    // Get depth data as array
    public float[,] GetDepthData()
    {
        if (depthReadback == null)
            depthReadback = new Texture2D(width, height, TextureFormat.RGB24, false);

        RenderTexture.active = depthTexture;
        depthReadback.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthReadback.Apply();

        Color[] pixels = depthReadback.GetPixels();
        float[,] depthData = new float[height, width];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int pixelIndex = y * width + x;
                float depthValue = pixels[pixelIndex].grayscale;
                // Convert normalized depth to actual distance
                depthData[y, x] = nearClip + depthValue * (farClip - nearClip);
            }
        }

        return depthData;
    }

    // Convert depth data to point cloud
    public Vector3[] DepthToPointCloud(float[,] depthData)
    {
        List<Vector3> points = new List<Vector3>();
        float fovRad = fieldOfView * Mathf.Deg2Rad;
        float aspectRatio = (float)width / height;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float depth = depthData[y, x];

                if (depth > nearClip && depth < farClip)
                {
                    // Calculate normalized device coordinates (-1 to 1)
                    float ndcX = (x / (float)width) * 2 - 1;
                    float ndcY = (1 - y / (float)height) * 2 - 1;

                    // Calculate world coordinates
                    float tanHalfFov = Mathf.Tan(fovRad / 2);
                    float xWorld = ndcX * tanHalfFov * aspectRatio * depth;
                    float yWorld = ndcY * tanHalfFov * depth;

                    Vector3 worldPoint = transform.position + transform.right * xWorld + transform.up * yWorld + transform.forward * depth;
                    points.Add(worldPoint);
                }
            }
        }

        return points.ToArray();
    }
}
```

### Noise Models and Calibration

Understanding and implementing realistic noise models for depth sensors.

#### Depth Camera Noise Models

Depth cameras exhibit several types of noise:
- Quantization noise: Discrete depth values
- Gaussian noise: Random measurement errors
- Multiplicative noise: Noise increases with distance
- Systematic errors: Calibration offsets

```xml
<!-- Adding noise to depth camera in Gazebo -->
<sensor name="noisy_depth_camera" type="depth">
  <!-- Previous configuration -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>  <!-- 1cm at 1m distance -->
  </noise>
</sensor>
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide information about acceleration and orientation, which are essential for robot localization and control.

### IMU Characteristics in Simulation

Simulating the various characteristics and noise profiles of real IMUs.

#### IMU Sensor Model in Gazebo

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <topic>__default_topic__</topic>
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

#### Unity IMU Simulation

```csharp
using UnityEngine;

public class IMUSimulator : MonoBehaviour
{
    [Header("IMU Configuration")]
    public float accelerometerNoise = 0.1f;
    public float gyroscopeNoise = 0.001f;
    public float magnetometerNoise = 0.1f;

    [Header("Calibration")]
    public Vector3 accelerometerBias = Vector3.zero;
    public Vector3 gyroscopeBias = Vector3.zero;
    public Vector3 magnetometerBias = Vector3.zero;

    private Rigidbody rb;
    private Vector3 lastPosition;
    private Quaternion lastRotation;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (!rb)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.isKinematic = true; // We'll control position manually
        }

        lastPosition = transform.position;
        lastRotation = transform.rotation;
    }

    void FixedUpdate()
    {
        // Calculate linear acceleration (approximate)
        Vector3 linearAcceleration = (transform.position - lastPosition - lastPosition + lastPosition) / (Time.fixedDeltaTime * Time.fixedDeltaTime);

        // Add noise to acceleration
        linearAcceleration += AddNoise(Vector3.one * accelerometerNoise);
        linearAcceleration += accelerometerBias;

        // Calculate angular velocity (approximate)
        Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastRotation);
        Vector3 angularVelocity = new Vector3(
            Mathf.Atan2(2 * (deltaRotation.w * deltaRotation.x + deltaRotation.y * deltaRotation.z),
                       1 - 2 * (deltaRotation.x * deltaRotation.x + deltaRotation.y * deltaRotation.y)) / Time.fixedDeltaTime,
            Mathf.Atan2(2 * (deltaRotation.w * deltaRotation.y - deltaRotation.z * deltaRotation.x),
                       1 - 2 * (deltaRotation.y * deltaRotation.y + deltaRotation.z * deltaRotation.z)) / Time.fixedDeltaTime,
            Mathf.Atan2(2 * (deltaRotation.w * deltaRotation.z + deltaRotation.x * deltaRotation.y),
                       1 - 2 * (deltaRotation.z * deltaRotation.z + deltaRotation.x * deltaRotation.x)) / Time.fixedDeltaTime
        );

        // Add noise to angular velocity
        angularVelocity += AddNoise(Vector3.one * gyroscopeNoise);
        angularVelocity += gyroscopeBias;

        // Simulate magnetometer (Earth's magnetic field in local frame)
        Vector3 magneticField = transform.InverseTransformDirection(Vector3.forward * 0.25f); // Approximate field
        magneticField += AddNoise(Vector3.one * magnetometerNoise);
        magneticField += magnetometerBias;

        // Store for next frame
        lastPosition = transform.position;
        lastRotation = transform.rotation;

        // Publish simulated IMU data
        PublishIMUData(linearAcceleration, angularVelocity, magneticField);
    }

    Vector3 AddNoise(Vector3 noiseLevels)
    {
        return new Vector3(
            Random.Range(-noiseLevels.x, noiseLevels.x),
            Random.Range(-noiseLevels.y, noiseLevels.y),
            Random.Range(-noiseLevels.z, noiseLevels.z)
        );
    }

    void PublishIMUData(Vector3 linearAcc, Vector3 angularVel, Vector3 magField)
    {
        // In a real implementation, this would publish to ROS/other systems
        Debug.Log($"IMU Data - Acc: {linearAcc}, Gyro: {angularVel}, Mag: {magField}");
    }
}
```

### Temperature and Drift Effects

Real IMUs exhibit temperature-dependent biases and drift over time:

```xml
<!-- Temperature effects in Gazebo IMU -->
<sensor name="thermal_imu" type="imu">
  <!-- Previous configuration -->
  <imu>
    <!-- Include thermal effects in noise model -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001</bias_stddev>
          <!-- Add thermal drift -->
          <dynamic_bias_stddev>0.00001</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>3600</dynamic_bias_correlation_time>
        </noise>
      </x>
    </angular_velocity>
  </imu>
</sensor>
```

## Sensor Fusion in Simulation

Combining data from multiple sensors to create a more accurate understanding of the robot's environment and state.

### Data Integration Techniques

Methods for combining sensor data in simulation environments.

#### Kalman Filter for Sensor Fusion

```csharp
using UnityEngine;

public class SensorFusionFilter : MonoBehaviour
{
    // State: [x, y, z, vx, vy, vz, ax, ay, az]
    private Matrix state; // 9x1
    private Matrix covariance; // 9x9

    private float processNoise = 0.1f;
    private float measurementNoise = 0.1f;

    void Start()
    {
        // Initialize state and covariance matrices
        state = new Matrix(9, 1);
        covariance = new Matrix(9, 9);

        // Initialize with identity matrix scaled by uncertainty
        for (int i = 0; i < 9; i++)
        {
            covariance[i, i] = 1.0f; // Initial uncertainty
        }
    }

    public void UpdateWithIMU(Vector3 linearAcc, Vector3 angularVel)
    {
        // Prediction step using IMU data
        Predict(linearAcc, angularVel);
    }

    public void UpdateWithLiDAR(Vector3 positionMeasurement)
    {
        // Correction step using LiDAR position
        Correct(positionMeasurement, 0, 1, 2); // x, y, z indices
    }

    public void UpdateWithDepthCamera(Vector3 positionMeasurement)
    {
        // Correction step using depth camera
        Correct(positionMeasurement, 0, 1, 2); // x, y, z indices
    }

    private void Predict(Vector3 linearAcc, Vector3 angularVel)
    {
        // State transition model (simplified)
        // Update position based on velocity and acceleration
        // Update velocity based on acceleration
        // Update acceleration based on IMU measurement

        // Jacobian of state transition function
        Matrix F = GetStateTransitionMatrix();

        // Process noise covariance
        Matrix Q = GetProcessNoiseCovariance();

        // Predict state: x_k = F * x_k-1
        state = F * state;

        // Predict covariance: P_k = F * P_k-1 * F^T + Q
        covariance = F * covariance * F.Transpose() + Q;
    }

    private void Correct(Vector3 measurement, params int[] measurementIndices)
    {
        // Measurement matrix H (maps state to measurement space)
        Matrix H = GetMeasurementMatrix(measurementIndices);

        // Innovation covariance
        Matrix S = H * covariance * H.Transpose() + GetMeasurementNoiseCovariance();

        // Kalman gain
        Matrix K = covariance * H.Transpose() * S.Inverse();

        // Innovation (difference between measurement and prediction)
        Vector3 predictedMeasurement = GetPredictedMeasurement();
        Matrix innovation = new Matrix(3, 1);
        innovation[0, 0] = measurement.x - predictedMeasurement.x;
        innovation[1, 0] = measurement.y - predictedMeasurement.y;
        innovation[2, 0] = measurement.z - predictedMeasurement.z;

        // Update state: x_k = x_k + K * innovation
        state = state + K * innovation;

        // Update covariance: P_k = (I - K * H) * P_k-1
        Matrix I = Matrix.Identity(9);
        covariance = (I - K * H) * covariance;
    }

    private Matrix GetStateTransitionMatrix()
    {
        // Simplified state transition matrix
        Matrix F = Matrix.Identity(9);

        // Fill in the appropriate values based on the motion model
        // This is a simplified example - a real implementation would be more complex

        return F;
    }

    private Matrix GetProcessNoiseCovariance()
    {
        Matrix Q = Matrix.Identity(9);
        for (int i = 0; i < 9; i++)
        {
            Q[i, i] = processNoise;
        }
        return Q;
    }

    private Matrix GetMeasurementNoiseCovariance()
    {
        Matrix R = Matrix.Identity(3);
        for (int i = 0; i < 3; i++)
        {
            R[i, i] = measurementNoise;
        }
        return R;
    }

    private Vector3 GetPredictedMeasurement()
    {
        // Extract position from state vector
        return new Vector3(state[0, 0], state[1, 0], state[2, 0]);
    }

    private Matrix GetMeasurementMatrix(int[] indices)
    {
        Matrix H = new Matrix(indices.Length, 9);
        for (int i = 0; i < indices.Length; i++)
        {
            H[i, indices[i]] = 1.0f;
        }
        return H;
    }
}
```

#### Particle Filter Implementation

For non-linear/non-Gaussian systems:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class ParticleFilter : MonoBehaviour
{
    [System.Serializable]
    public class Particle
    {
        public Vector3 position;
        public Vector3 velocity;
        public float weight;

        public Particle(Vector3 pos, Vector3 vel, float w)
        {
            position = pos;
            velocity = vel;
            weight = w;
        }
    }

    public List<Particle> particles = new List<Particle>();
    public int particleCount = 100;

    void Start()
    {
        InitializeParticles();
    }

    void InitializeParticles()
    {
        particles.Clear();
        float weight = 1.0f / particleCount;

        for (int i = 0; i < particleCount; i++)
        {
            Vector3 pos = transform.position + Random.insideUnitSphere * 0.5f; // Small uncertainty
            Vector3 vel = Random.insideUnitSphere * 0.1f; // Small initial velocity
            particles.Add(new Particle(pos, vel, weight));
        }
    }

    public void Predict(Vector3 controlInput, float deltaTime)
    {
        foreach (var particle in particles)
        {
            // Apply motion model with noise
            Vector3 noise = Random.insideUnitSphere * 0.01f; // Process noise
            particle.position += particle.velocity * deltaTime + noise;
            particle.velocity += controlInput * deltaTime;
        }
    }

    public void UpdateWeights(Vector3 measurement, float measurementNoise)
    {
        float totalWeight = 0f;

        foreach (var particle in particles)
        {
            // Calculate likelihood of measurement given particle state
            float dist = Vector3.Distance(particle.position, measurement);
            float likelihood = Mathf.Exp(-(dist * dist) / (2 * measurementNoise * measurementNoise));
            particle.weight *= likelihood;
            totalWeight += particle.weight;
        }

        // Normalize weights
        if (totalWeight > 0)
        {
            foreach (var particle in particles)
            {
                particle.weight /= totalWeight;
            }
        }
    }

    public void Resample()
    {
        List<Particle> newParticles = new List<Particle>();
        float weightSum = 0f;

        // Calculate cumulative weights
        List<float> cumulativeWeights = new List<float>();
        foreach (var particle in particles)
        {
            weightSum += particle.weight;
            cumulativeWeights.Add(weightSum);
        }

        // Resample particles
        for (int i = 0; i < particleCount; i++)
        {
            float randomValue = Random.value * weightSum;
            int index = 0;

            // Find particle corresponding to random value
            for (int j = 0; j < cumulativeWeights.Count; j++)
            {
                if (randomValue <= cumulativeWeights[j])
                {
                    index = j;
                    break;
                }
            }

            // Add particle with uniform weight
            Particle selectedParticle = particles[index];
            Vector3 noise = Random.insideUnitSphere * 0.001f; // Small resampling noise
            newParticles.Add(new Particle(
                selectedParticle.position + noise,
                selectedParticle.velocity,
                1.0f / particleCount
            ));
        }

        particles = newParticles;
    }

    public Vector3 GetEstimatedState()
    {
        Vector3 weightedSum = Vector3.zero;
        float totalWeight = 0f;

        foreach (var particle in particles)
        {
            weightedSum += particle.position * particle.weight;
            totalWeight += particle.weight;
        }

        if (totalWeight > 0)
        {
            return weightedSum / totalWeight;
        }
        else
        {
            return transform.position; // Fallback to current position
        }
    }
}
```

## Practical Examples

### Example 1: Configuring a Multi-Sensor Robot in Gazebo

Complete example of a humanoid robot with multiple sensors:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="humanoid_with_sensors">
    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <izz>0.5</izz>
        </inertia>
      </inertial>

      <!-- Visual and collision -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.8</size>
          </box>
        </geometry>
      </collision>

      <!-- IMU Sensor -->
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <pose>0 0 0.2 0 0 0</pose>
        <topic>imu/data</topic>
        <visualize>false</visualize>
      </sensor>

      <!-- Depth Camera -->
      <sensor name="depth_camera" type="depth">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <pose>0.1 0 0.3 0 0 0</pose>
        <topic>camera/depth</topic>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
      </sensor>

      <!-- 2D LiDAR -->
      <sensor name="lidar_2d" type="ray">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <pose>0.1 0 0.4 0 0 0</pose>
        <topic>lidar/scan</topic>
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </link>

    <!-- Head link with additional sensors -->
    <link name="head_link">
      <pose>0 0 1.5 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>

      <!-- 3D LiDAR on head -->
      <sensor name="lidar_3d" type="ray">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <pose>0 0 0.1 0 0 0</pose>
        <topic>lidar_3d/scan</topic>
        <ray>
          <scan>
            <horizontal>
              <samples>1440</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>64</samples>
              <resolution>1</resolution>
              <min_angle>-0.5236</min_angle>
              <max_angle>0.1745</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>120.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </link>

    <!-- Joint between base and head -->
    <joint name="neck_joint" type="revolute">
      <parent>base_link</parent>
      <child>head_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
      <pose>0 0 0.4 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### Example 2: Sensor Data Processing Pipeline

Unity script for processing multiple sensor inputs:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorDataProcessor : MonoBehaviour
{
    [Header("Sensor References")]
    public LiDARSimulator lidar;
    public DepthCameraSimulator depthCam;
    public IMUSimulator imu;

    [Header("Fusion Settings")]
    public float positionTrustFactor = 0.7f; // Higher trust in position sensors
    public float orientationTrustFactor = 0.9f; // Higher trust in IMU for orientation

    private Vector3 estimatedPosition;
    private Quaternion estimatedOrientation;
    private ParticleFilter particleFilter;
    private SensorFusionFilter kalmanFilter;

    void Start()
    {
        particleFilter = GetComponent<ParticleFilter>() ?? gameObject.AddComponent<ParticleFilter>();
        kalmanFilter = GetComponent<SensorFusionFilter>() ?? gameObject.AddComponent<SensorFusionFilter>();

        estimatedPosition = transform.position;
        estimatedOrientation = transform.rotation;
    }

    void Update()
    {
        ProcessSensorData();
    }

    void ProcessSensorData()
    {
        // Get sensor measurements
        Vector3[] lidarPoints = lidar?.GetPointCloud() ?? new Vector3[0];
        float[,] depthData = depthCam?.GetDepthData() ?? null;
        // IMU data is processed internally by the IMUSimulator

        // Process LiDAR data
        if (lidarPoints.Length > 0)
        {
            Vector3 lidarPosition = EstimatePositionFromLiDAR(lidarPoints);
            particleFilter.UpdateWeights(lidarPosition, 0.2f); // 20cm uncertainty
        }

        // Process depth camera data
        if (depthData != null)
        {
            Vector3 depthPosition = EstimatePositionFromDepth(depthData);
            particleFilter.UpdateWeights(depthPosition, 0.3f); // 30cm uncertainty
        }

        // Resample particles based on weights
        particleFilter.Resample();

        // Get fused estimate
        estimatedPosition = particleFilter.GetEstimatedState();
        estimatedOrientation = GetFusedOrientation(); // Combine with IMU data

        // Update robot pose
        transform.position = estimatedPosition;
        transform.rotation = estimatedOrientation;
    }

    Vector3 EstimatePositionFromLiDAR(Vector3[] points)
    {
        // Simple centroid calculation for demonstration
        if (points.Length == 0) return transform.position;

        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }

        return sum / points.Length;
    }

    Vector3 EstimatePositionFromDepth(float[,] depthData)
    {
        // Convert depth data to position estimate
        // This is a simplified approach - real implementation would be more complex
        int height = depthData.GetLength(0);
        int width = depthData.GetLength(1);

        List<Vector3> validPoints = new List<Vector3>();
        float threshold = 5.0f; // Ignore points beyond 5m

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (depthData[y, x] < threshold)
                {
                    // Convert pixel coordinates to world coordinates
                    // This requires camera intrinsic parameters
                    float worldX = (x - width / 2.0f) * depthData[y, x] * 0.001f; // Rough conversion
                    float worldY = (height / 2.0f - y) * depthData[y, x] * 0.001f;
                    float worldZ = depthData[y, x];

                    Vector3 worldPoint = new Vector3(worldX, worldY, worldZ);
                    validPoints.Add(transform.TransformPoint(worldPoint));
                }
            }
        }

        if (validPoints.Count > 0)
        {
            Vector3 sum = Vector3.zero;
            foreach (Vector3 point in validPoints)
            {
                sum += point;
            }
            return sum / validPoints.Count;
        }

        return transform.position;
    }

    Quaternion GetFusedOrientation()
    {
        // In a real implementation, this would combine IMU data with other sources
        // For now, we'll just return the current rotation
        return transform.rotation;
    }

    // Method to get fused sensor data
    public SensorData GetFusedData()
    {
        return new SensorData
        {
            position = estimatedPosition,
            orientation = estimatedOrientation,
            lidarPoints = lidar?.GetPointCloud(),
            timestamp = Time.time
        };
    }

    [System.Serializable]
    public struct SensorData
    {
        public Vector3 position;
        public Quaternion orientation;
        public Vector3[] lidarPoints;
        public float timestamp;
    }
}
```

## Sample Sensor Data Outputs and Validation

### LiDAR Point Cloud Format

LiDAR sensors typically output data in PointCloud2 format:

```
# Example PointCloud2 message structure
header:
  seq: 1234
  stamp:
    secs: 1234
    nsecs: 567890
  frame_id: "lidar_frame"
height: 1
width: 720
fields:
  - name: "x"
    offset: 0
    datatype: 7  # FLOAT32
    count: 1
  - name: "y"
    offset: 4
    datatype: 7  # FLOAT32
    count: 1
  - name: "z"
    offset: 8
    datatype: 7  # FLOAT32
    count: 1
is_bigendian: False
point_step: 16  # 4 bytes per float * 4 fields (x,y,z,intensity)
row_step: 11520  # 16 bytes * 720 points
data: [720*16 bytes of binary data...]
is_dense: False
```

### Depth Image Format

Depth cameras output images in various formats:

```
# Example depth image message
header:
  seq: 1234
  stamp:
    secs: 1234
    nsecs: 567890
  frame_id: "camera_frame"
height: 480
width: 640
encoding: "32FC1"  # 32-bit float, 1 channel
is_bigendian: False
step: 2560  # 640 pixels * 4 bytes per float
data: [640*480*4 bytes of binary depth data...]
```

## Exercises for Students

1. Create a Gazebo world with a humanoid robot equipped with LiDAR, depth camera, and IMU sensors
2. Implement a simple SLAM algorithm using the simulated sensor data
3. Compare the performance of different sensor fusion techniques (Kalman filter vs. particle filter)
4. Add noise to the simulated sensors and analyze its effect on perception accuracy
5. Create a simulation scenario where the robot must navigate using only sensor data
6. Implement a sensor validation system that detects and filters out anomalous sensor readings
7. Design a sensor placement strategy that maximizes perception effectiveness for humanoid navigation