:github_url: https://github.com/edxmorgan/uvms-simulator/blob/main/doc/userdoc.rst

.. _ros2_control_RA5BHS_userdoc:

************************************************
Reach Alpha Blue Heavy Dynamics Simulator
************************************************

The *Reach Alpha Blue Heavy Dynamics Simulator* includes an interface plugin that supports multiple state and command interfaces.

Prerequisites
--------------------------

Before beginning the tutorial steps, ensure you have installed the necessary packages and dependencies:

.. code-block:: shell

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
   https://github.com/casadi/casadi/wiki/InstallationLinux

   # Initialize Git LFS and pull the necessary files for dynamics and kinematics
   git lfs install
   git lfs pull

Tutorial Steps
--------------------------

1. **Verify the Simulator Descriptions**

   To check that the *Reach Alpha Blue Heavy Sim* descriptions are working correctly, use the following launch command:

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 view_robot.launch.py

2. **Start the Reach Alpha 5 Example**

   Open a terminal, source your ROS2 workspace, and execute the launch file with:

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py

   Useful launch-file options:

   - ``use_manipulator_hardware:=false``: Starts the simulator and connects to a real reach alpha hardware manipulator in the loop.

   - ``use_vehicle_hardware:=false``: Starts the simulator and connects to a real bluerov2 heavy underwater vehicle in the loop.

   - ``sim_robot_count:=n``: Starts the simulator by spawning n number of underwater vehicle manipulator systems. Example; ``sim_robot_count:=4``

   The launch file will load and start the robot hardware, controllers, and open *RViz*. You will see extensive output from the hardware implementation in the terminal, showing its internal states.

3. **Verify Running Controllers**

   To check which controllers are currently active, run:

   .. code-block:: shell

    ros2 control list_controllers

   The output should look like:

   .. code-block:: shell

      fts_broadcaster_1   [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
      fts_broadcaster_3   [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      imu_broadcaster_2   [imu_sensor_broadcaster/IMUSensorBroadcaster] active    
      imu_broadcaster_3   [imu_sensor_broadcaster/IMUSensorBroadcaster] active    
      imu_broadcaster_real[imu_sensor_broadcaster/IMUSensorBroadcaster] active    
      fts_broadcaster_2   [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      imu_broadcaster_4   [imu_sensor_broadcaster/IMUSensorBroadcaster] active    
      imu_broadcaster_1   [imu_sensor_broadcaster/IMUSensorBroadcaster] active    
      fts_broadcaster_4   [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
      forward_effort_controller[forward_command_controller/ForwardCommandController] active

   Observe how this output changes based on the launch file arguments used.

.. 5. **Send Commands to the Controller**

..    If the controllers are active, you can send commands to the *Forward Current Controller* as follows:

..    - For the ``forward_current_controller``:

..      .. code-block:: shell

..       ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

..    - For the ``forward_effort_controller``:

..      .. code-block:: shell

..       ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

..    .. note::
..       The first five floating-point values correspond to the manipulator, from the base at index[0] to the end-effector at index[4]. The following eight values are for the vehicle's thrusters.
