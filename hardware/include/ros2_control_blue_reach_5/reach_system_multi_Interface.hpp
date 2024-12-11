// Copyright 2021 Department of Engineering Cybernetics, NTNU
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_BLUE_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <random>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"

#include "ros2_control_blue_reach_5/driver.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
#include "ros2_control_blue_reach_5/joint.hpp"
#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/dynamics.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <casadi/casadi.hpp>

namespace ros2_control_blue_reach_5
{

  using RefType = std_msgs::msg::Float64MultiArray;
  class ReachSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      // Parameters for the RRBot simulation
      std::string serial_port_;
      int state_update_freq_;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ReachSystemMultiInterfaceHardware);

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    // ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    // hardware_interface::return_type perform_command_mode_switch(
    //     const std::vector<std::string> &start_interfaces,
    //     const std::vector<std::string> &stop_interfaces) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ROS2_CONTROL_BLUE_REACH_5_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // ReachComms comms_;
    Config cfg_;

    enum class mode_level_t
    {
      MODE_STANDBY,
      MODE_DISABLE,
      MODE_POSITION,
      MODE_VELOCITY,
      MODE_CURRENT,
      MODE_EFFORT,
      MODE_CARTESIAN
    };

    // Active control mode for each actuator
    std::vector<mode_level_t> control_level_;

    double predictor_i = 0;

    // Store the state & commands for the robot joints
    std::vector<Joint> hw_joint_struct_;

    // Declare the vectors before the if-else block
    std::vector<DM> q_prev;
    std::vector<DM> q_dot_prev;

    std::vector<DM> q_prev_mhe;
    std::vector<DM> q_dot_prev_mhe;

    std::vector<DM> C2T_arg;
    std::vector<DM> T2C_arg;

    std::vector<DM> forward_p0;
    std::vector<DM> backward_p0;
    std::vector<DM> FD_param_Selector_arg;
    std::vector<DM> FD_selected_p0;

    std::vector<DM> p_mhe = {1e-05, 1e-05, 1e-05, 1e-05, 5.0, 2.3, 2.2, 0.3, 5.0, 1.8, 1.0, 1.15};
    // std::vector<DM> p_mhe = {1e-05, 1e-05, 1e-05, 1e-05,  1.0, 1.0, 1.0, 1.0,   1.0, 1.0, 1.0, 1.0};

    // std::vector<DM> FD_param_x = {1e-05, 1e-05, 1e-05, 1e-05, 3, 1.6, 1.8, 0.3, 3, 2, 0.8, 1.5};
    std::vector<DM> FD_param_x = {1e-05, 1e-05, 1e-05, 1e-05, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // std::vector<DM> FD_param_x = {1e-05, 1e-05, 1e-05, 1e-05, 4, 1.6, 2.8, 0.7, 3.4, 2.2, 1.2, 1.8};

    std::vector<double> drag;

    std::vector<DM> fd_arg;
    std::vector<DM> FNEXT;

    std::vector<DM> fd_arg_mhe;
    std::vector<DM> FNEXT_mhe;

    rclcpp::Subscription<RefType>::SharedPtr topic_based_parameter_subscriber_;
    rclcpp::Node::SharedPtr node_;
    RefType latest_parameter_state_;

    double delta_seconds;
    double time_seconds;
    // Store the dynamics function for the robot joints
    casadi_reach_alpha_5::Dynamics dynamics_service;

    // std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_;
    // std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    //     realtime_odometry_transform_publisher_;

    /**
     * @brief Write the current position of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The position packet that signaled the callback.
     */
    void updatePositionCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Write the current velocity of the robot received from the serial client to the
     * respective asynchronous vector.
     *
     * @param packet The velocity packet that signaled the callback.
     */
    void updateVelocityCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Asynchronously read the current state of the robot by polling the robot serial
     * interface.
     *
     * @param freq The frequency (Hz) that the interface should poll the current robot state at.
     */

    void updateCurrentCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_structs_ref);

    /**
     * @brief Asynchronously read the current state of the robot by polling the robot serial
     * interface.
     *
     * @param freq The frequency (Hz) that the interface should poll the current robot state at.
     */

    void pollState(int freq) const;

    // Driver things
    alpha::driver::Driver driver_;
    std::thread state_request_worker_;
    std::atomic<bool> running_{false};

    std::mutex access_async_states_;

    bool excite = false;
  };

} // namespace ros2_control_blue_reach_5
#endif // ROS2_CONTROL_BLUE_REACH_5__REACH_SYSTEM_MULTI_INTERFACE_HPP_
