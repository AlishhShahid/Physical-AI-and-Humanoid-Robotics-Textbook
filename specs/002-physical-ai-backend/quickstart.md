# Quickstart

This guide will help you get started with the Physical AI Backend & Humanoid Robotics Control Server project.

## Prerequisites

*   Python 3.8+
*   Docker (for containerized deployment)
*   ROS2 Foxy Fitzroy (if interacting with physical/simulated robots locally)
*   NVIDIA Docker (for GPU-accelerated environments like Isaac ROS)

## Setup

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-username/humanoid-robotics-textbook.git
    cd humanoid-robotics-textbook
    ```

2.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

3.  **Setup Python virtual environment**:
    ```bash
    python -m venv .venv
    source .venv/bin/activate  # On Windows, use `.venv\Scripts\activate`
    pip install -r requirements.txt
    ```

4.  **Run the FastAPI application (development)**:
    ```bash
    uvicorn src.main:app --reload
    ```
    The API documentation will be available at `http://127.0.0.1:8000/docs`.

5.  **Build and run with Docker**:
    ```bash
    docker build -t physical-ai-backend .
    docker run -p 8000:8000 physical-ai-backend
    ```

6.  **Deployment on NVIDIA Jetson**:
    Refer to the `scripts/deploy_jetson.sh` for deployment instructions on a Jetson Orin device.
