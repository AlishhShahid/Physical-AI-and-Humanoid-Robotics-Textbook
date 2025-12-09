# API Contracts

This document outlines the REST API contracts for the Physical AI Backend.

## 1. RAG-based Queries

*   **Endpoint**: `/rag/query`
*   **Method**: `POST`
*   **Description**: Submits a query to the RAG system and retrieves relevant information from indexed documents.
*   **Request Body**:
    ```json
    {
        "query": "string"
    }
    ```
*   **Response Body**:
    ```json
    {
        "answer": "string",
        "sources": [
            {
                "filename": "string",
                "section": "string"
            }
        ]
    }
    ```
*   **Status Codes**:
    *   `200 OK`: Successful query.
    *   `400 Bad Request`: Invalid query format.
    *   `500 Internal Server Error`: Server-side error.

## 2. Command Submission (Text-based)

*   **Endpoint**: `/command/text`
*   **Method**: `POST`
*   **Description**: Submits a text command for the robot to execute. The backend processes this command through the VLA model.
*   **Request Body**:
    ```json
    {
        "command": "string"
    }
    ```
*   **Response Body**:
    ```json
    {
        "command_id": "string",
        "status": "string"
    }
    ```
*   **Status Codes**:
    *   `202 Accepted`: Command received and being processed.
    *   `400 Bad Request`: Invalid command format or safety violation.
    *   `500 Internal Server Error`: Server-side error.

## 3. Teleoperation Override

*   **Endpoint**: `/teleop/override`
*   **Method**: `POST`
*   **Description**: Activates or deactivates teleoperation override mode.
*   **Request Body**:
    ```json
    {
        "enable": "boolean"
    }
    ```
*   **Response Body**:
    ```json
    {
        "status": "string",
        "message": "string"
    }
    ```
*   **Status Codes**:
    *   `200 OK`: Teleoperation mode changed successfully.
    *   `500 Internal Server Error`: Server-side error.
