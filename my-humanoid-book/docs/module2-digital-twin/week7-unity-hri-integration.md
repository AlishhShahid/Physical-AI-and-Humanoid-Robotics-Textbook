---
sidebar_position: 2
---

# Week 7: Unity Human-Robot Interaction Integration

## Learning Objectives

By the end of this week, you will be able to:
- Set up Unity for humanoid robot visualization and interaction
- Implement ROS2 communication bridges for Unity integration
- Create intuitive user interfaces for robot control and monitoring
- Develop virtual reality (VR) interfaces for immersive HRI
- Implement safety and feedback systems in Unity environments

## Introduction to Unity for Humanoid Robotics

Unity is a powerful game engine that can be leveraged for creating immersive human-robot interaction (HRI) experiences. When combined with ROS2, Unity enables rich visualization, intuitive control interfaces, and immersive VR experiences for humanoid robot teleoperation and monitoring.

### Unity ROS2 Integration Architecture

The integration between Unity and ROS2 typically involves:

1. **ROS2 TCP Bridge**: Facilitates communication between Unity and ROS2
2. **Message Serialization**: Converting Unity data structures to ROS2 messages
3. **Visualization System**: Rendering robot models and sensor data
4. **Input System**: Processing user input for robot control
5. **Simulation Synchronization**: Keeping Unity visualization in sync with ROS2 simulation

## Setting Up Unity for ROS2 Integration

First, let's establish the basic Unity project structure for ROS2 communication:

```csharp
// Assets/Scripts/ROS2/ROS2Connector.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2ForUnity.Messages.Std_msgs;
using Ros2ForUnity.Messages.Sensor_msgs;
using Ros2ForUnity.Messages.Geometry_msgs;
using Ros2Sharp;

public class ROS2Connector : MonoBehaviour
{
    [Header("ROS2 Configuration")]
    public string rosAgentIP = "127.0.0.1";
    public int rosAgentPort = 8888;

    [Header("Robot Configuration")]
    public string robotNamespace = "/humanoid_robot";

    private Ros2Socket ros2Socket;
    private bool isConnected = false;

    // Publishers
    private Publisher<std_msgs.msg.String> statusPub;
    private Publisher<geometry_msgs.msg.Twist> cmdVelPub;
    private Publisher<std_msgs.msg.Float64MultiArray> jointCmdPub;

    // Subscribers
    private Subscriber<sensor_msgs.msg.JointState> jointStateSub;
    private Subscriber<sensor_msgs.msg.Imu> imuSub;
    private Subscriber<nav_msgs.msg.Odometry> odomSub;

    // Robot data
    private JointStateMsg currentJointState;
    private ImuMsg currentImuData;
    private OdometryMsg currentOdomData;

    void Start()
    {
        InitializeROS2Connection();
    }

    void InitializeROS2Connection()
    {
        try
        {
            // Initialize ROS2 socket
            ros2Socket = new Ros2Socket();
            ros2Socket.Init(rosAgentIP, rosAgentPort);

            // Create publishers
            statusPub = ros2Socket.Advertise<std_msgs.msg.String>($"{robotNamespace}/unity_status");
            cmdVelPub = ros2Socket.Advertise<geometry_msgs.msg.Twist>($"{robotNamespace}/cmd_vel");
            jointCmdPub = ros2Socket.Advertise<std_msgs.msg.Float64MultiArray>($"{robotNamespace}/joint_commands");

            // Create subscribers
            jointStateSub = ros2Socket.Subscribe<sensor_msgs.msg.JointState>(
                $"{robotNamespace}/joint_states", OnJointStateReceived);
            imuSub = ros2Socket.Subscribe<sensor_msgs.msg.Imu>(
                $"{robotNamespace}/imu/data", OnImuReceived);
            odomSub = ros2Socket.Subscribe<nav_msgs.msg.Odometry>(
                $"{robotNamespace}/odom", OnOdomReceived);

            isConnected = true;
            Debug.Log("ROS2 connection established");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to ROS2: {e.Message}");
            isConnected = false;
        }
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        currentJointState = msg;
        UpdateRobotVisualization();
    }

    void OnImuReceived(ImuMsg msg)
    {
        currentImuData = msg;
    }

    void OnOdomReceived(OdometryMsg msg)
    {
        currentOdomData = msg;
        UpdateRobotPosition(msg);
    }

    void Update()
    {
        if (isConnected)
        {
            // Send periodic status updates
            if (Time.time % 5.0f < Time.deltaTime) // Every 5 seconds
            {
                SendStatusUpdate();
            }
        }
    }

    void SendStatusUpdate()
    {
        var statusMsg = new std_msgs.msg.String();
        statusMsg.Data = $"Unity client active - Time: {Time.time}";
        statusPub.Publish(statusMsg);
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        if (isConnected)
        {
            var cmd = new geometry_msgs.msg.Twist();
            cmd.Linear.X = linearX;
            cmd.Linear.Y = 0.0f;
            cmd.Linear.Z = 0.0f;
            cmd.Angular.X = 0.0f;
            cmd.Angular.Y = 0.0f;
            cmd.Angular.Z = angularZ;

            cmdVelPub.Publish(cmd);
        }
    }

    public void SendJointCommands(float[] jointPositions)
    {
        if (isConnected)
        {
            var cmd = new std_msgs.msg.Float64MultiArray();
            cmd.Data = new List<double>();
            foreach (float pos in jointPositions)
            {
                cmd.Data.Add((double)pos);
            }

            jointCmdPub.Publish(cmd);
        }
    }

    void UpdateRobotVisualization()
    {
        // This method will be called when joint state is received
        // Implement robot model updates here
    }

    void UpdateRobotPosition(OdometryMsg odom)
    {
        // Update robot position in Unity based on odometry
        transform.position = new Vector3(
            (float)odom.Pose.Pose.Position.X,
            (float)odom.Pose.Pose.Position.Z, // Unity Y is up, ROS Z is up
            (float)odom.Pose.Pose.Position.Y  // Unity Z is forward, ROS Y is lateral
        );
    }

    void OnDestroy()
    {
        if (ros2Socket != null)
        {
            ros2Socket.Dispose();
        }
    }
}
```

## Robot Visualization in Unity

Creating an accurate robot model visualization is crucial for effective HRI:

```csharp
// Assets/Scripts/Robot/RobotVisualizer.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2ForUnity.Messages.Sensor_msgs;

public class RobotVisualizer : MonoBehaviour
{
    [Header("Robot Joint Links")]
    public Transform baseLink;
    public Transform leftHipLink;
    public Transform rightHipLink;
    public Transform leftKneeLink;
    public Transform rightKneeLink;
    public Transform leftAnkleLink;
    public Transform rightAnkleLink;
    public Transform leftShoulderLink;
    public Transform rightShoulderLink;
    public Transform leftElbowLink;
    public Transform rightElbowLink;
    public Transform headLink;

    [Header("Joint Configuration")]
    public List<string> jointNames;
    public List<Transform> jointTransforms;
    public List<float> jointMinLimits;
    public List<float> jointMaxLimits;

    private Dictionary<string, int> jointIndexMap;
    private float[] currentJointPositions;

    void Start()
    {
        InitializeJointMapping();
        currentJointPositions = new float[jointNames.Count];
    }

    void InitializeJointMapping()
    {
        jointIndexMap = new Dictionary<string, int>();
        for (int i = 0; i < jointNames.Count; i++)
        {
            jointIndexMap[jointNames[i]] = i;
        }
    }

    public void UpdateRobotPose(JointStateMsg jointState)
    {
        if (jointState == null || jointState.Position.Count == 0)
            return;

        // Update joint positions array
        for (int i = 0; i < jointState.Name.Count && i < jointState.Position.Count; i++)
        {
            string jointName = jointState.Name[i];
            if (jointIndexMap.ContainsKey(jointName))
            {
                int index = jointIndexMap[jointName];
                float position = (float)jointState.Position[i];

                // Apply safety limits
                position = Mathf.Clamp(position, jointMinLimits[index], jointMaxLimits[index]);
                currentJointPositions[index] = position;
            }
        }

        // Apply joint positions to transforms
        ApplyJointPositions();
    }

    void ApplyJointPositions()
    {
        // Apply positions to each joint transform
        for (int i = 0; i < jointNames.Count && i < currentJointPositions.Length; i++)
        {
            string jointName = jointNames[i];
            float position = currentJointPositions[i];

            Transform jointTransform = GetJointTransformByName(jointName);
            if (jointTransform != null)
            {
                // Apply rotation based on joint type
                ApplyJointRotation(jointTransform, jointName, position);
            }
        }
    }

    Transform GetJointTransformByName(string jointName)
    {
        switch (jointName)
        {
            case "left_hip_joint": return leftHipLink;
            case "right_hip_joint": return rightHipLink;
            case "left_knee_joint": return leftKneeLink;
            case "right_knee_joint": return rightKneeLink;
            case "left_ankle_joint": return leftAnkleLink;
            case "right_ankle_joint": return rightAnkleLink;
            case "left_shoulder_joint": return leftShoulderLink;
            case "right_shoulder_joint": return rightShoulderLink;
            case "left_elbow_joint": return leftElbowLink;
            case "right_elbow_joint": return rightElbowLink;
            case "neck_joint": return headLink;
            default: return null;
        }
    }

    void ApplyJointRotation(Transform jointTransform, string jointName, float position)
    {
        // Define rotation axes based on joint type
        Vector3 rotationAxis = Vector3.zero;

        if (jointName.Contains("hip") || jointName.Contains("shoulder"))
        {
            rotationAxis = Vector3.right; // X-axis rotation
        }
        else if (jointName.Contains("knee") || jointName.Contains("elbow"))
        {
            rotationAxis = Vector3.right; // X-axis rotation
        }
        else if (jointName.Contains("ankle"))
        {
            rotationAxis = Vector3.forward; // Z-axis rotation
        }
        else if (jointName.Contains("neck"))
        {
            rotationAxis = Vector3.right; // X-axis rotation
        }

        // Apply rotation
        jointTransform.localRotation = Quaternion.AngleAxis(Mathf.Rad2Deg * position, rotationAxis);
    }

    // Helper method to get current joint positions
    public float[] GetCurrentJointPositions()
    {
        return currentJointPositions;
    }
}
```

## Human-Robot Interaction Interfaces

Creating intuitive interfaces is essential for effective HRI:

```csharp
// Assets/Scripts/HRI/ControlInterface.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class ControlInterface : MonoBehaviour
{
    [Header("Movement Controls")]
    public Slider linearSpeedSlider;
    public Slider angularSpeedSlider;
    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;
    public Button stopButton;

    [Header("Joint Control")]
    public List<Slider> jointSliders;
    public Button homePositionButton;
    public Button walkPoseButton;
    public Button balancePoseButton;

    [Header("Visualization")]
    public Toggle showTrajectoryToggle;
    public Toggle showSensorsToggle;
    public Toggle showSafetyZonesToggle;

    [Header("References")]
    public ROS2Connector ros2Connector;
    public RobotVisualizer robotVisualizer;
    public Camera mainCamera;

    private float linearSpeed = 0.5f;
    private float angularSpeed = 0.5f;

    void Start()
    {
        SetupEventHandlers();
        SetupDefaultValues();
    }

    void SetupEventHandlers()
    {
        // Movement controls
        forwardButton.onClick.AddListener(() => MoveRobot(1, 0));
        backwardButton.onClick.AddListener(() => MoveRobot(-1, 0));
        leftButton.onClick.AddListener(() => MoveRobot(0, 1));
        rightButton.onClick.AddListener(() => MoveRobot(0, -1));
        stopButton.onClick.AddListener(() => StopRobot());

        // Speed controls
        linearSpeedSlider.onValueChanged.AddListener(OnLinearSpeedChanged);
        angularSpeedSlider.onValueChanged.AddListener(OnAngularSpeedChanged);

        // Joint controls
        homePositionButton.onClick.AddListener(SetHomePosition);
        walkPoseButton.onClick.AddListener(SetWalkPose);
        balancePoseButton.onClick.AddListener(SetBalancePose);

        // Visualization toggles
        showTrajectoryToggle.onValueChanged.AddListener(OnTrajectoryToggleChanged);
        showSensorsToggle.onValueChanged.AddListener(OnSensorsToggleChanged);
        showSafetyZonesToggle.onValueChanged.AddListener(OnSafetyZonesToggleChanged);
    }

    void SetupDefaultValues()
    {
        linearSpeedSlider.value = 0.5f;
        angularSpeedSlider.value = 0.5f;
    }

    void MoveRobot(int linearDir, int angularDir)
    {
        if (ros2Connector != null)
        {
            float linear = linearDir * linearSpeed;
            float angular = angularDir * angularSpeed;
            ros2Connector.SendVelocityCommand(linear, angular);
        }
    }

    void StopRobot()
    {
        if (ros2Connector != null)
        {
            ros2Connector.SendVelocityCommand(0, 0);
        }
    }

    void OnLinearSpeedChanged(float value)
    {
        linearSpeed = value;
    }

    void OnAngularSpeedChanged(float value)
    {
        angularSpeed = value;
    }

    void SetHomePosition()
    {
        // Set all joints to home position (0)
        float[] homePositions = new float[robotVisualizer.GetCurrentJointPositions().Length];
        for (int i = 0; i < homePositions.Length; i++)
        {
            homePositions[i] = 0.0f;
        }

        if (ros2Connector != null)
        {
            ros2Connector.SendJointCommands(homePositions);
        }
    }

    void SetWalkPose()
    {
        // Set joints to a walking-ready pose
        float[] walkPositions = new float[robotVisualizer.GetCurrentJointPositions().Length];

        // Example walk-ready pose (hips slightly bent, knees ready)
        if (robotVisualizer.jointIndexMap.ContainsKey("left_hip_joint"))
            walkPositions[robotVisualizer.jointIndexMap["left_hip_joint"]] = 0.1f;
        if (robotVisualizer.jointIndexMap.ContainsKey("right_hip_joint"))
            walkPositions[robotVisualizer.jointIndexMap["right_hip_joint"]] = -0.1f;
        if (robotVisualizer.jointIndexMap.ContainsKey("left_knee_joint"))
            walkPositions[robotVisualizer.jointIndexMap["left_knee_joint"]] = 0.2f;
        if (robotVisualizer.jointIndexMap.ContainsKey("right_knee_joint"))
            walkPositions[robotVisualizer.jointIndexMap["right_knee_joint"]] = -0.2f;

        if (ros2Connector != null)
        {
            ros2Connector.SendJointCommands(walkPositions);
        }
    }

    void SetBalancePose()
    {
        // Set joints to a balance-ready pose
        float[] balancePositions = new float[robotVisualizer.GetCurrentJointPositions().Length];

        // Example balance pose (slightly crouched, arms out for stability)
        if (robotVisualizer.jointIndexMap.ContainsKey("left_hip_joint"))
            balancePositions[robotVisualizer.jointIndexMap["left_hip_joint"]] = 0.2f;
        if (robotVisualizer.jointIndexMap.ContainsKey("right_hip_joint"))
            balancePositions[robotVisualizer.jointIndexMap["right_hip_joint"]] = -0.2f;
        if (robotVisualizer.jointIndexMap.ContainsKey("left_knee_joint"))
            balancePositions[robotVisualizer.jointIndexMap["left_knee_joint"]] = 0.3f;
        if (robotVisualizer.jointIndexMap.ContainsKey("right_knee_joint"))
            balancePositions[robotVisualizer.jointIndexMap["right_knee_joint"]] = -0.3f;
        if (robotVisualizer.jointIndexMap.ContainsKey("left_shoulder_joint"))
            balancePositions[robotVisualizer.jointIndexMap["left_shoulder_joint"]] = 0.5f;
        if (robotVisualizer.jointIndexMap.ContainsKey("right_shoulder_joint"))
            balancePositions[robotVisualizer.jointIndexMap["right_shoulder_joint"]] = -0.5f;

        if (ros2Connector != null)
        {
            ros2Connector.SendJointCommands(balancePositions);
        }
    }

    void OnTrajectoryToggleChanged(bool isOn)
    {
        // Toggle trajectory visualization
        // Implementation depends on trajectory visualization system
    }

    void OnSensorsToggleChanged(bool isOn)
    {
        // Toggle sensor visualization
        // Implementation depends on sensor visualization system
    }

    void OnSafetyZonesToggleChanged(bool isOn)
    {
        // Toggle safety zone visualization
        // Implementation depends on safety zone visualization system
    }

    // Method to update joint sliders based on current joint state
    public void UpdateJointSliders(float[] jointPositions)
    {
        for (int i = 0; i < jointSliders.Count && i < jointPositions.Length; i++)
        {
            if (i < jointSliders.Count)
            {
                jointSliders[i].value = jointPositions[i];
            }
        }
    }

    void Update()
    {
        // Handle keyboard controls
        HandleKeyboardControls();
    }

    void HandleKeyboardControls()
    {
        if (Input.GetKeyDown(KeyCode.W)) MoveRobot(1, 0); // Forward
        if (Input.GetKeyDown(KeyCode.S)) MoveRobot(-1, 0); // Backward
        if (Input.GetKeyDown(KeyCode.A)) MoveRobot(0, 1); // Left
        if (Input.GetKeyDown(KeyCode.D)) MoveRobot(0, -1); // Right
        if (Input.GetKeyDown(KeyCode.Space)) StopRobot(); // Stop
        if (Input.GetKeyDown(KeyCode.H)) SetHomePosition(); // Home position
        if (Input.GetKeyDown(KeyCode.Tab)) SetWalkPose(); // Walk pose
    }
}
```

## VR Integration for Immersive HRI

For advanced HRI, Unity supports VR integration:

```csharp
// Assets/Scripts/VR/VRController.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class VRController : MonoBehaviour
{
    [Header("VR Configuration")]
    public bool enableVR = true;
    public Transform leftController;
    public Transform rightController;
    public Camera vrCamera;

    [Header("Teleoperation")]
    public bool enableTeleoperation = true;
    public float teleopSensitivity = 1.0f;

    [Header("Safety")]
    public float maxTeleopDistance = 5.0f;
    public bool enableSafetyChecks = true;

    private XRNode leftHand = XRNode.LeftHand;
    private XRNode rightHand = XRNode.RightHand;
    private InputDevice leftControllerDevice;
    private InputDevice rightControllerDevice;

    void Start()
    {
        if (enableVR)
        {
            InitializeVR();
        }
    }

    void InitializeVR()
    {
        // Initialize VR input devices
        var inputDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(leftHand, inputDevices);
        if (inputDevices.Count > 0)
        {
            leftControllerDevice = inputDevices[0];
        }

        inputDevices.Clear();
        InputDevices.GetDevicesAtXRNode(rightHand, inputDevices);
        if (inputDevices.Count > 0)
        {
            rightControllerDevice = inputDevices[0];
        }
    }

    void Update()
    {
        if (enableVR)
        {
            UpdateVRControllers();
            HandleVRInput();
        }
    }

    void UpdateVRControllers()
    {
        // Update controller positions and rotations
        if (leftControllerDevice.isValid)
        {
            Vector3 position;
            Quaternion rotation;

            if (leftControllerDevice.TryGetFeatureValue(CommonUsages.devicePosition, out position))
            {
                leftController.position = position;
            }

            if (leftControllerDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation))
            {
                leftController.rotation = rotation;
            }
        }

        if (rightControllerDevice.isValid)
        {
            Vector3 position;
            Quaternion rotation;

            if (rightControllerDevice.TryGetFeatureValue(CommonUsages.devicePosition, out position))
            {
                rightController.position = position;
            }

            if (rightControllerDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation))
            {
                rightController.rotation = rotation;
            }
        }
    }

    void HandleVRInput()
    {
        if (enableTeleoperation)
        {
            HandleVRTeleoperation();
        }
    }

    void HandleVRTeleoperation()
    {
        // Get controller poses
        Vector3 leftPos = leftController.position;
        Vector3 rightPos = rightController.position;

        // Calculate desired robot movement based on controller positions
        Vector3 movementVector = (rightPos - leftPos) * teleopSensitivity;

        // Apply safety checks
        if (enableSafetyChecks)
        {
            movementVector = Vector3.ClampMagnitude(movementVector, maxTeleopDistance);
        }

        // Send movement command to robot
        if (Mathf.Abs(movementVector.x) > 0.1f || Mathf.Abs(movementVector.z) > 0.1f)
        {
            // Convert Unity coordinates to ROS coordinates
            float linearX = movementVector.z;  // Forward/backward
            float angularZ = -movementVector.x; // Left/right rotation

            if (GetComponent<ROS2Connector>() != null)
            {
                GetComponent<ROS2Connector>().SendVelocityCommand(linearX, angularZ);
            }
        }
    }

    // Method to handle VR grab interactions
    public void HandleVRGrabInteraction()
    {
        bool leftTriggerPressed = false;
        bool rightTriggerPressed = false;

        if (leftControllerDevice.isValid)
        {
            leftControllerDevice.TryGetFeatureValue(CommonUsages.triggerButton, out leftTriggerPressed);
        }

        if (rightControllerDevice.isValid)
        {
            rightControllerDevice.TryGetFeatureValue(CommonUsages.triggerButton, out rightTriggerPressed);
        }

        // Handle grab interactions based on trigger presses
        if (leftTriggerPressed || rightTriggerPressed)
        {
            // Implement object grabbing logic
            HandleObjectGrab(leftTriggerPressed, rightTriggerPressed);
        }
    }

    void HandleObjectGrab(bool leftGrab, bool rightGrab)
    {
        // Implementation for grabbing objects in VR
        // This would involve raycasting from controllers to detect grabbable objects
    }
}
```

## Safety and Feedback Systems

Implementing safety systems is crucial for HRI applications:

```csharp
// Assets/Scripts/Safety/SafetySystem.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SafetySystem : MonoBehaviour
{
    [Header("Safety Parameters")]
    public float maxLinearVelocity = 1.0f;
    public float maxAngularVelocity = 1.0f;
    public float emergencyStopDistance = 0.5f;
    public bool enableEmergencyStop = true;

    [Header("Safety Visualization")]
    public GameObject safetyZoneIndicator;
    public Material safeMaterial;
    public Material warningMaterial;
    public Material dangerMaterial;

    [Header("Emergency Controls")]
    public Button emergencyStopButton;
    public Text safetyStatusText;

    private bool isEmergencyActive = false;
    private bool isRobotMoving = false;

    void Start()
    {
        SetupSafetySystem();
    }

    void SetupSafetySystem()
    {
        if (emergencyStopButton != null)
        {
            emergencyStopButton.onClick.AddListener(TriggerEmergencyStop);
        }

        // Initialize safety zone indicators
        UpdateSafetyVisualization();
    }

    void Update()
    {
        CheckSafetyConditions();
        UpdateSafetyVisualization();
    }

    void CheckSafetyConditions()
    {
        // Check if robot is moving beyond safe parameters
        isRobotMoving = CheckRobotMovement();

        if (isRobotMoving && enableEmergencyStop)
        {
            // Check for emergency conditions
            CheckEmergencyConditions();
        }
    }

    bool CheckRobotMovement()
    {
        // This would check actual robot movement data
        // For simulation, we'll return true if any movement command was recently sent
        return true; // Simplified for example
    }

    void CheckEmergencyConditions()
    {
        // Check various emergency conditions
        if (CheckProximitySensors())
        {
            HandleEmergencyCondition("Proximity Alert");
        }

        if (CheckJointLimits())
        {
            HandleEmergencyCondition("Joint Limit Exceeded");
        }

        if (CheckBalance())
        {
            HandleEmergencyCondition("Balance Lost");
        }
    }

    bool CheckProximitySensors()
    {
        // Simulate proximity sensor checking
        // In real implementation, this would check sensor data
        return false; // No proximity alert in simulation
    }

    bool CheckJointLimits()
    {
        // Check if any joints are approaching limits
        if (GetComponent<RobotVisualizer>() != null)
        {
            float[] jointPositions = GetComponent<RobotVisualizer>().GetCurrentJointPositions();
            for (int i = 0; i < jointPositions.Length; i++)
            {
                if (Mathf.Abs(jointPositions[i]) > 0.95f) // 95% of limit
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool CheckBalance()
    {
        // Check if robot is losing balance
        // This would use IMU data in real implementation
        return false; // No balance issue in simulation
    }

    void HandleEmergencyCondition(string condition)
    {
        Debug.LogWarning($"Safety System: Emergency condition detected - {condition}");

        // Trigger emergency stop
        TriggerEmergencyStop();

        // Update UI
        if (safetyStatusText != null)
        {
            safetyStatusText.text = $"EMERGENCY: {condition}";
            safetyStatusText.color = Color.red;
        }
    }

    public void TriggerEmergencyStop()
    {
        isEmergencyActive = true;

        // Send emergency stop command to robot
        if (GetComponent<ROS2Connector>() != null)
        {
            GetComponent<ROS2Connector>().SendVelocityCommand(0, 0);
        }

        // Update UI
        if (safetyStatusText != null)
        {
            safetyStatusText.text = "EMERGENCY STOP ACTIVATED";
            safetyStatusText.color = Color.red;
        }

        Debug.Log("Emergency stop activated - all robot movement halted");
    }

    public void ClearEmergencyStop()
    {
        isEmergencyActive = false;

        // Update UI
        if (safetyStatusText != null)
        {
            safetyStatusText.text = "SAFE";
            safetyStatusText.color = Color.green;
        }

        Debug.Log("Emergency stop cleared");
    }

    void UpdateSafetyVisualization()
    {
        if (safetyZoneIndicator != null)
        {
            if (isEmergencyActive)
            {
                safetyZoneIndicator.GetComponent<Renderer>().material = dangerMaterial;
            }
            else if (isRobotMoving)
            {
                safetyZoneIndicator.GetComponent<Renderer>().material = warningMaterial;
            }
            else
            {
                safetyZoneIndicator.GetComponent<Renderer>().material = safeMaterial;
            }
        }
    }

    // Method to check if movement is safe
    public bool IsMovementSafe(float linearVel, float angularVel)
    {
        if (Mathf.Abs(linearVel) > maxLinearVelocity || Mathf.Abs(angularVel) > maxAngularVelocity)
        {
            return false;
        }

        return !isEmergencyActive;
    }
}
```

## Practical Exercise: Unity HRI Implementation

1. Set up a Unity project with ROS2 integration
2. Create a humanoid robot model with proper joint hierarchy
3. Implement ROS2 communication for joint state and command exchange
4. Develop intuitive control interfaces for robot operation
5. Add VR support for immersive teleoperation
6. Implement safety systems with visual feedback
7. Test the complete HRI system with simulated robot data

## Summary

This week, we've covered Unity integration for human-robot interaction:

- **ROS2 communication setup** for Unity-ROS2 bridge
- **Robot visualization** with accurate joint state updates
- **Human-robot interaction interfaces** for intuitive control
- **VR integration** for immersive teleoperation experiences
- **Safety and feedback systems** for secure HRI

Unity provides a powerful platform for creating engaging and intuitive interfaces for humanoid robot control and monitoring, making complex robotic systems more accessible to human operators.