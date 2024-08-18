:github_url: https://github.com/RKinDLab/Reach_alpha_Blue_heavy_Sim/blob/master/doc/userdoc.rst

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

   .. note::
    It is normal to see the message ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``. This warning appears because the ``joint_state_publisher_gui`` node needs a moment to start. The ``joint_state_publisher_gui`` provides a GUI to generate a random configuration for the robot, which will be displayed in *RViz*.

2. **Start the Reach Alpha 5 Example**

   Open a terminal, source your ROS2 workspace, and execute the launch file with:

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py use_mock_hardware:=true

   Useful launch-file options:

   - ``robot_controller:=forward_effort_controller``: Starts the demo and spawns an effort controller, allowing the robot to be controlled using the ``forward_effort_controller``.

   - ``robot_controller:=forward_current_controller``: Starts the demo and spawns a current controller, allowing the robot to be controlled using the ``forward_current_controller``.

   The launch file will load and start the robot hardware, controllers, and open *RViz*. You will see extensive output from the hardware implementation in the terminal, showing its internal states.

3. **Check Hardware Interface**

   Verify that the hardware interface has loaded properly by running:

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   .. code-block:: shell

      command interfaces
         alpha_axis_a/current [available] [claimed]
         alpha_axis_a/effort [available] [unclaimed]
         alpha_axis_a/position [available] [unclaimed]
         alpha_axis_a/velocity [available] [unclaimed]
         alpha_axis_b/current [available] [claimed]
         alpha_axis_b/effort [available] [unclaimed]
         alpha_axis_b/position [available] [unclaimed]
         alpha_axis_b/velocity [available] [unclaimed]
         alpha_axis_c/current [available] [claimed]
         alpha_axis_c/effort [available] [unclaimed]
         alpha_axis_c/position [available] [unclaimed]
         alpha_axis_c/velocity [available] [unclaimed]
         alpha_axis_d/current [available] [claimed]
         alpha_axis_d/effort [available] [unclaimed]
         alpha_axis_d/position [available] [unclaimed]
         alpha_axis_d/velocity [available] [unclaimed]
         alpha_axis_e/current [available] [claimed]
         alpha_axis_e/effort [available] [unclaimed]
         alpha_axis_e/position [available] [unclaimed]
         alpha_axis_e/velocity [available] [unclaimed]
         alphathruster1_joint/current [available] [claimed]
         alphathruster1_joint/effort [available] [unclaimed]
         alphathruster2_joint/current [available] [claimed]
         alphathruster2_joint/effort [available] [unclaimed]
         alphathruster3_joint/current [available] [claimed]
         alphathruster3_joint/effort [available] [unclaimed]
         alphathruster4_joint/current [available] [claimed]
         alphathruster4_joint/effort [available] [unclaimed]
         alphathruster5_joint/current [available] [claimed]
         alphathruster5_joint/effort [available] [unclaimed]
         alphathruster6_joint/current [available] [claimed]
         alphathruster6_joint/effort [available] [unclaimed]
         alphathruster7_joint/current [available] [claimed]
         alphathruster7_joint/effort [available] [unclaimed]
         alphathruster8_joint/current [available] [claimed]
         alphathruster8_joint/effort [available] [unclaimed]

      state interfaces
         alpha_axis_a/acceleration
         alpha_axis_a/current
         alpha_axis_a/effort
         alpha_axis_a/estimated_acceleration
         alpha_axis_a/estimated_effort
         alpha_axis_a/estimated_inertia_zz
         alpha_axis_a/filtered_position
         alpha_axis_a/filtered_velocity
         alpha_axis_a/position
         alpha_axis_a/stateId
         alpha_axis_a/velocity
         alpha_axis_b/acceleration
         alpha_axis_b/current
         alpha_axis_b/effort
         alpha_axis_b/estimated_acceleration
         alpha_axis_b/estimated_effort
         alpha_axis_b/estimated_inertia_zz
         alpha_axis_b/filtered_position
         alpha_axis_b/filtered_velocity
         alpha_axis_b/position
         alpha_axis_b/stateId
         alpha_axis_b/velocity
         alpha_axis_c/acceleration
         alpha_axis_c/current
         alpha_axis_c/effort
         alpha_axis_c/estimated_acceleration
         alpha_axis_c/estimated_effort
         alpha_axis_c/estimated_inertia_zz
         alpha_axis_c/filtered_position
         alpha_axis_c/filtered_velocity
         alpha_axis_c/position
         alpha_axis_c/stateId
         alpha_axis_c/velocity
         alpha_axis_d/acceleration
         alpha_axis_d/current
         alpha_axis_d/effort
         alpha_axis_d/estimated_acceleration
         alpha_axis_d/estimated_effort
         alpha_axis_d/estimated_inertia_zz
         alpha_axis_d/filtered_position
         alpha_axis_d/filtered_velocity
         alpha_axis_d/position
         alpha_axis_d/stateId
         alpha_axis_d/velocity
         alpha_axis_e/acceleration
         alpha_axis_e/current
         alpha_axis_e/effort
         alpha_axis_e/estimated_acceleration
         alpha_axis_e/estimated_effort
         alpha_axis_e/estimated_inertia_zz
         alpha_axis_e/filtered_position
         alpha_axis_e/filtered_velocity
         alpha_axis_e/position
         alpha_axis_e/stateId
         alpha_axis_e/velocity
         alphaimu_sensor/orientation.w
         alphaimu_sensor/orientation.x
         alphaimu_sensor/orientation.y
         alphaimu_sensor/orientation.z
         alphaimu_sensor/position.x
         alphaimu_sensor/position.y
         alphaimu_sensor/position.z
         alphaimu_sensor/velocity.p
         alphaimu_sensor/velocity.q
         alphaimu_sensor/velocity.r
         alphaimu_sensor/velocity.u
         alphaimu_sensor/velocity.v
         alphaimu_sensor/velocity.w
         alphathruster1_joint/acceleration
         alphathruster1_joint/current
         alphathruster1_joint/effort
         alphathruster1_joint/position
         alphathruster1_joint/velocity
         alphathruster2_joint/acceleration
         alphathruster2_joint/current
         alphathruster2_joint/effort
         alphathruster2_joint/position
         alphathruster2_joint/velocity
         alphathruster3_joint/acceleration
         alphathruster3_joint/current
         alphathruster3_joint/effort
         alphathruster3_joint/position
         alphathruster3_joint/velocity
         alphathruster4_joint/acceleration
         alphathruster4_joint/current
         alphathruster4_joint/effort
         alphathruster4_joint/position
         alphathruster4_joint/velocity
         alphathruster5_joint/acceleration
         alphathruster5_joint/current
         alphathruster5_joint/effort
         alphathruster5_joint/position
         alphathruster5_joint/velocity
         alphathruster6_joint/acceleration
         alphathruster6_joint/current
         alphathruster6_joint/effort
         alphathruster6_joint/position
         alphathruster6_joint/velocity
         alphathruster7_joint/acceleration
         alphathruster7_joint/current
         alphathruster7_joint/effort
         alphathruster7_joint/position
         alphathruster7_joint/velocity
         alphathruster8_joint/acceleration
         alphathruster8_joint/current
         alphathruster8_joint/effort
         alphathruster8_joint/position
         alphathruster8_joint/velocity

   Markings of ``[claimed]`` by command interfaces indicate that a controller has access to the command system.

4. **Verify Running Controllers**

   To check which controllers are currently active, run:

   .. code-block:: shell

    ros2 control list_controllers

   The output should look like:

   .. code-block:: shell

      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
      forward_current_controller[forward_command_controller/ForwardCommandController] active

   Observe how this output changes based on the launch file arguments used.

5. **Send Commands to the Controller**

   If the controllers are active, you can send commands to the *Forward Current Controller* as follows:

   - For the ``forward_current_controller``:

     .. code-block:: shell

      ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

   - For the ``forward_effort_controller``:

     .. code-block:: shell

      ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

   .. note::
      The first five floating-point values correspond to the manipulator, from the base at index[0] to the end-effector at index[4]. The following eight values are for the vehicle's thrusters.
