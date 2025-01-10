# Underwater Vehicle and Manipulator Simulator

A simulator for the **BlueROV Heavy** equipped with a **Reach Alpha 5** manipulator based on the `ros2_control` framework. Integrates Thor Fossen’s underwater dynamics with Featherstone's manipulator dynamics algorithm for realistic simulations.

---

## Features

- **Realistic Dynamics:** Accurate simulation of underwater vehicle and manipulator behaviors.
- **Multi-Agent Support:** Simulate multiple agents within a shared environment.
- **Hardware-in-the-Loop Support:** Integrates BlueROV Heavy hardware including IMU, A50 DVL, and Reach Alpha 5 manipulator for realistic interaction and testing.
- **Video Demonstration:** [![Watch the Video](https://img.youtube.com/vi/VRJUbpdvPIM/0.jpg)](https://www.youtube.com/watch?v=VRJUbpdvPIM)

---

## Getting Started
- [ROS2 installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Prerequisites

Ensure the following dependencies are installed:

```bash
sudo apt-get install git-lfs
sudo apt-get install ros-humble-hardware-interface
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-joint-state-broadcaster
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-forward-command-controller
sudo apt-get install ros-humble-ros2-control

# Install CasADi (required for dynamics and kinematics calculations)
# Follow the installation instructions on the CasADi wiki:
# https://github.com/casadi/casadi/wiki/InstallationLinux

# Initialize Git LFS and pull the necessary files for dynamics and kinematics
git lfs install
git lfs pull
```

Clone additional packages to your ROS2 workspace:

- [uvms_dynamics_ros2_control](https://github.com/edxmorgan/uvms_dynamics_ros2_control)
- [uvms_interfaces](https://github.com/edxmorgan/uvms_interfaces/tree/main)
- [sensor/GPIO transform broadcaster](https://github.com/edxmorgan/tf2_broadcaster)

### Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/edxmorgan/uvms-simulator.git
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

---

## Dynamics Foundation

This simulator incorporates and extends:

- Vehicle dynamics from [diff_uv](https://github.com/edxmorgan/diff_uv).
- Unified UVMS dynamics from [diff_uvms](https://github.com/edxmorgan/diff_uvms).

---

## Documentation

For detailed setup and usage instructions, refer to the [User Documentation](doc/userdoc.rst).

---

## Online Resources

- [ROS Control](https://control.ros.org/rolling/index.html)
- [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master)
- [Blue Robotics](https://github.com/Bluerobotics)

---

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for enhancements or bug fixes.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Hardware-in-the-Loop Setup

To enable Hardware-in-the-Loop (HIL) simulation with the BlueROV Heavy, follow these additional steps:

1. **Connect Hardware Components:**
   - **IMU:** Ensure the Inertial Measurement Unit (IMU) is properly connected to your system.
   - **A50 DVL:** Connect the Doppler Velocity Log (DVL) to interface with the simulator.
   - **Reach Alpha 5 Manipulator:** Integrate the manipulator hardware following the [Reach Robotics SDK](https://github.com/Reach-Robotics/reach_robotics_sdk/tree/master) guidelines.

2. **Configure ROS2 Nodes:**
   - Launch the appropriate ROS2 nodes to interface with each hardware component.
   - Update the simulation configuration files to include hardware topics and parameters.

3. **Calibrate Sensors:**
   - Perform calibration for the IMU and DVL to ensure accurate data representation within the simulator.

4. **Testing:**
   - Run the simulator and verify that real-time data from the hardware components is accurately reflected in the simulation environment.
   - Use the provided controllers to manipulate the Reach Alpha 5 and observe interactions within the simulated underwater environment.

For detailed instructions on setting up HIL, refer to the [Hardware-in-the-Loop Documentation](doc/hil_setup.rst).

---