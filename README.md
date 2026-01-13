# Robocon ROS 2 Project

Welcome to the Robocon ROS 2 project! This workspace contains the simulation and control software for our robot, designed for ROS 2 Jazzy and Gazebo Harmonic.

## 🚀 Getting Started

We use **Docker** and **VS Code Dev Containers** to ensure everyone has the same development environment. You don't need to install ROS 2 on your local machine!

### Prerequisites
- [Docker Desktop](https://www.docker.com/products/docker-desktop)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for VS Code

### 📦 Installation

1.  **Clone the repository** (if you haven't already):
    ```bash
    git clone <your-repo-url>
    cd robocon
    ```
2.  **Open in VS Code**:
    ```bash
    code .
    ```
3.  **Reopen in Container**:
    - When asked "Reopen in Container", click **Reopen**.
    - Or press `F1` and select **Dev Containers: Reopen in Container**.
    - *Wait for the container to build (this may take a few minutes the first time).*

## 🛠️ Build and Run

Once inside the VS Code terminal (you should see `root@...`):

1.  **Build the workspace**:
    ```bash
    colcon build
    ```

2.  **Source the environment**:
    ```bash
    source install/setup.bash
    ```

3.  **Run the Simulation**:
    ```bash
    ros2 launch robocon_bringup sim.launch.py
    ```
    This will open Gazebo with the Robocon 2026 game field and the robot model.

## 📂 Project Structure

The code is organized in `src/` following the `ros_gz_project_template` pattern:

*   **`robocon_description`**: Robot models (URDF/SDF), meshes, and visual assets.
*   **`robocon_gazebo`**: Game field worlds (`robocon_2026.sdf`) and simulation settings.
*   **`robocon_application`**: The "brain" of the robot. Contains:
    *   `robocon_application/robot_logic_node.py`: Python node for mode switching and logic.
    *   `config/localization.yaml`: Parameters for localization (AMCL/EKF).
*   **`robocon_bringup`**: Launch files to start everything.

## 🤝 Contributing

- Always run `colcon build` to check for errors before committing.
- New models go into `robocon_description/models`.
- New worlds go into `robocon_gazebo/worlds`.

## ❓ Troubleshooting

- **"Package not found"**: Did you run `source install/setup.bash`?
- **Gazebo models missing**: Ensure `robocon_description` is built and sourced so the `GZ_SIM_RESOURCE_PATH` is set correctly.
