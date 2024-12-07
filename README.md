# Reach Alpha Blue Heavy Dynamics Simulator

A `ros2_control` simulator for the **BlueROV Heavy** equipped with a **Reach Alpha 5** manipulator. Integrates Thor Fossen’s underwater dynamics with Featherstone's manipulator dynamics algorithm for realistic simulations.

## Features
- **Realistic Dynamics:** Accurate simulation of underwater and manipulator behaviors.
- **Multi-Agent Support:** Simulate multiple agents within a shared environment.
- **Video Demonstration:** [![Watch the Video](https://img.youtube.com/vi/VRJUbpdvPIM/0.jpg)](https://www.youtube.com/watch?v=VRJUbpdvPIM)

## Getting Started

### Prerequisites
- **ROS2**: Ensure you have ROS2 installed. [Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/blue-heavy-dynamics-simulator.git
    ```
2. Navigate to the project directory:
    ```bash
    cd blue-heavy-dynamics-simulator
    ```
3. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
4. Build the workspace:
    ```bash
    colcon build
    ```
5. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Documentation
For detailed setup and usage instructions, refer to the [User Documentation](doc/userdoc.rst).

## Online Resources
Explore extended ROS2 control capabilities on [ROS Control](https://control.ros.org/rolling/index.html).


## Disclaimer
This project is independently developed and not affiliated with **Reach Robotics** or **Blue Robotics**. For official software and support, visit:
- [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
- [Blue Robotics](https://github.com/Bluerobotics)

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License
[MIT License](LICENSE)
