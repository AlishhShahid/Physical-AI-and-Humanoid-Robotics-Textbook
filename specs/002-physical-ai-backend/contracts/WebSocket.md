# WebSocket Contracts

This document outlines the WebSocket communication contracts for the Physical AI Backend.

## 1. Robot Control Messages

*   **Endpoint**: `/ws/robot_control`
*   **Description**: Real-time communication channel for streaming robot control messages and receiving robot status updates.
*   **Outgoing Messages (Server to Client)**:
    *   **Robot Status Update**:
        ```json
        {
            "type": "robot_status",
            "robot_id": "string",
            "status": "string",
            "current_pose": { "x": float, "y": float, "z": float, "orientation": "string" },
            "timestamp": "datetime"
        }
        ```
    *   **Action Plan Status**:
        ```json
        {
            "type": "action_plan_status",
            "command_id": "string",
            "action_id": "string",
            "status": "string",
            "message": "string",
            "timestamp": "datetime"
        }
        ```
*   **Incoming Messages (Client to Server)**:
    *   **Direct Control Command**:
        ```json
        {
            "type": "direct_control",
            "robot_id": "string",
            "command_type": "string",
            "payload": { "key": "value" }
        }
        ```

## 2. Sensor Data Stream

*   **Endpoint**: `/ws/sensor_data`
*   **Description**: Real-time streaming of sensor data from the robot (e.g., camera feeds, IMU data).
*   **Outgoing Messages (Server to Client)**:
    *   **Camera Feed**:
        ```json
        {
            "type": "camera_feed",
            "robot_id": "string",
            "camera_id": "string",
            "image_data": "base64_encoded_image",
            "timestamp": "datetime"
        }
        ```
    *   **IMU Data**:
        ```json
        {
            "type": "imu_data",
            "robot_id": "string",
            "imu_id": "string",
            "accelerometer": { "x": float, "y": float, "z": float },
            "gyroscope": { "x": float, "y": float, "z": float },
            "timestamp": "datetime"
        }
        ```
