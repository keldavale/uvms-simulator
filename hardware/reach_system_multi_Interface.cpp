#include "ros2_control_blue_reach_5/reach_system_multi_Interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "ros2_control_blue_reach_5/device_id.hpp"
#include "ros2_control_blue_reach_5/mode.hpp"
#include "ros2_control_blue_reach_5/packet_id.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{

  std::string vectorToString(const std::vector<DM> &vec)
  {
    std::ostringstream oss;
    for (const auto &elem : vec)
    {
      oss << elem << " "; // Adjust this to correctly format DM objects
    }
    return oss.str();
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Print the CasADi version
    std::string casadi_version = CasadiMeta::version();
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "CasADi version: %s", casadi_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
    // Use CasADi's "external" to load the compiled dynamics functions
    dynamics_service.usage_cplusplus_checks("test", "libtest.so", "reach system");
    dynamics_service.kalman_filter = dynamics_service.load_casadi_fun("KF_PREDICT", "libKFnext.so");
    dynamics_service.current2torqueMap = dynamics_service.load_casadi_fun("current_to_torque_map", "libC2T.so");
    dynamics_service.torque2currentMap = dynamics_service.load_casadi_fun("torque_to_current_map", "libT2C.so");

    dynamics_service.inverse_dynamics = dynamics_service.load_casadi_fun("tau_eval", "libTau_MEval.so");

    // dynamics_service.forward_dynamics = dynamics_service.load_casadi_fun("stochastic_model", "libAFnext.so");
    // dynamics_service.pd_controller = dynamics_service.load_casadi_fun("pid", "libPD.so");

    cfg_.serial_port_ = info_.hardware_parameters["serial_port"];
    cfg_.state_update_freq_ = std::stoi(info_.hardware_parameters["state_update_frequency"]);

    robot_structs_.hw_joint_struct_.reserve(info_.joints.size());

    // hw_joint_struct_.reserve(info_.joints.size());
    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Hardware update rate is %u Hz", static_cast<int>(cfg_.state_update_freq_));

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));
      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));
      RCLCPP_INFO(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));

      double max_effort = stod(joint.parameters.at("max_effort"));
      bool positionLimitsFlag = stoi(joint.parameters.at("has_position_limits"));
      double min_position = stod(joint.parameters.at("min_position"));
      double max_position = stod(joint.parameters.at("max_position"));
      double max_velocity = stod(joint.parameters.at("max_velocity"));
      double soft_k_position = stod(joint.parameters.at("soft_k_position"));
      double soft_k_velocity = stod(joint.parameters.at("soft_k_velocity"));
      double soft_min_position = stod(joint.parameters.at("soft_min_position"));
      double soft_max_position = stod(joint.parameters.at("soft_max_position"));
      double kt = stod(joint.parameters.at("kt"));
      double forward_I_static = stod(joint.parameters.at("forward_I_static"));
      double backward_I_static = stod(joint.parameters.at("backward_I_static"));

      Joint::State initialState{default_position, 0.0, 0.0};
      Joint::Limits jointLimits{.position_min = min_position, .position_max = max_position, .velocity_max = max_velocity, .effort_max = max_effort};
      Joint::SoftLimits jointSoftLimits{.position_k = soft_k_position, .velocity_k = soft_k_velocity, .position_min = soft_min_position, .position_max = soft_max_position};
      Joint::MotorInfo actuatorProp{.kt = kt, .forward_I_static = forward_I_static, .backward_I_static = backward_I_static};
      robot_structs_.hw_joint_struct_.emplace_back(joint.name, device_id, initialState, jointLimits, positionLimitsFlag, jointSoftLimits, actuatorProp);
      // ReachSystemMultiInterface has exactly 13 state interfaces
      // and 4 command interfaces on each joint

      if (joint.command_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s' has %zu command interfaces. 4 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 19)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "Joint '%s'has %zu state interfaces. 19 expected.",
            joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    };

    hardware_interface::ComponentInfo endeffector_IO = info_.gpios[0];
    if (endeffector_IO.state_interfaces.size() != 7)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
          "GPIO '%s'has %zu state interfaces. 7 expected.", endeffector_IO.name.c_str(),
          endeffector_IO.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    };

    hardware_interface::ComponentInfo step_IO = info_.gpios[1];
    if (step_IO.state_interfaces.size() != 17)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
          "GPIO '%s'has %zu state interfaces. 17 expected.", step_IO.name.c_str(),
          step_IO.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    };

    // Add random ID to prevent warnings about multiple publishers within the same node
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name});
    node_ = rclcpp::Node::make_shared("_", options);
    topic_based_parameter_subscriber_ = node_->create_subscription<RefType>("/viscous_drag", rclcpp::SensorDataQoS(),
                                                                            [this](const RefType::SharedPtr joint_state)
                                                                            { latest_parameter_state_ = *joint_state; });

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
  {
    // Start the driver
    try
    {
      driver_.start(cfg_.serial_port_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL( // NOLINT
          rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
          "Failed to configure the serial driver for the AlphaHardware system interface.");

      return hardware_interface::CallbackReturn::ERROR;
    }

    // Register callbacks for joint states
    driver_.subscribe(
        alpha::driver::PacketId::PacketID_POSITION,
        [this](const alpha::driver::Packet &packet) -> void
        { updatePositionCb(packet, robot_structs_.hw_joint_struct_); });

    driver_.subscribe(
        alpha::driver::PacketId::PacketID_VELOCITY,
        [this](const alpha::driver::Packet &packet) -> void
        { updateVelocityCb(packet, robot_structs_.hw_joint_struct_); });

    driver_.subscribe(
        alpha::driver::PacketId::PacketID_CURRENT,
        [this](const alpha::driver::Packet &packet) -> void
        { updateCurrentCb(packet, robot_structs_.hw_joint_struct_); });

    // Start a thread to request state updates
    running_.store(true);
    state_request_worker_ = std::thread(&ReachSystemMultiInterfaceHardware::pollState, this, cfg_.state_update_freq_);

    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
        "Successfully configured the ReachSystemMultiInterfaceHardware system interface for serial communication!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

    running_.store(false);
    state_request_worker_.join();
    driver_.stop();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "preparing command mode switch");
    // Prepare for new command modes
    std::vector<mode_level_t> new_modes = {};

    for (std::string key : start_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {

        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          new_modes.push_back(mode_level_t::MODE_POSITION);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          new_modes.push_back(mode_level_t::MODE_VELOCITY);
        }
        if (key == info_.joints[i].name + "/" + custom_hardware_interface::HW_IF_CURRENT)
        {
          new_modes.push_back(mode_level_t::MODE_CURRENT);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
        {
          new_modes.push_back(mode_level_t::MODE_EFFORT);
        }
      }
    };

    //  criteria: All joints must be given new command mode at the same time
    if (new_modes.size() != info_.joints.size())
    {
      return hardware_interface::return_type::ERROR;
    };

    //  criteria: All joints must have the same command mode
    if (!std::all_of(
            new_modes.begin() + 1, new_modes.end(),
            [&](mode_level_t mode)
            { return mode == new_modes[0]; }))
    {
      return hardware_interface::return_type::ERROR;
    }

    // Stop motion on all relevant joints that are stopping
    for (std::string key : stop_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          robot_structs_.hw_joint_struct_[i].command_state_.velocity = 0;
          robot_structs_.hw_joint_struct_[i].command_state_.current = 0;
          control_level_[i] = mode_level_t::MODE_DISABLE; // Revert to undefined
        }
      }
    }

    // Set the new command modes
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (control_level_[i] != mode_level_t::MODE_DISABLE)
      {
        // Something else is using the joint! Abort!
        return hardware_interface::return_type::ERROR;
      }
      control_level_[i] = new_modes[i];
    }
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Command Mode Switch successful");
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  ReachSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &robot_structs_.hw_joint_struct_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &robot_structs_.hw_joint_struct_[i].current_state_.filtered_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &robot_structs_.hw_joint_struct_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &robot_structs_.hw_joint_struct_[i].current_state_.filtered_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &robot_structs_.hw_joint_struct_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &robot_structs_.hw_joint_struct_[i].current_state_.estimated_acceleration));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &robot_structs_.hw_joint_struct_[i].current_state_.current));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &robot_structs_.hw_joint_struct_[i].current_state_.effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &robot_structs_.hw_joint_struct_[i].current_state_.computed_effort));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT_UNCERTAINTY, &robot_structs_.hw_joint_struct_[i].current_state_.computed_effort_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION, &robot_structs_.hw_joint_struct_[i].current_state_.predicted_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION_UNCERTAINTY, &robot_structs_.hw_joint_struct_[i].current_state_.predicted_position_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY, &robot_structs_.hw_joint_struct_[i].current_state_.predicted_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY_UNCERTAINTY, &robot_structs_.hw_joint_struct_[i].current_state_.predicted_velocity_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION, &robot_structs_.hw_joint_struct_[i].current_state_.adaptive_predicted_position));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY, &robot_structs_.hw_joint_struct_[i].current_state_.adaptive_predicted_position_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY, &robot_structs_.hw_joint_struct_[i].current_state_.adaptive_predicted_velocity));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY, &robot_structs_.hw_joint_struct_[i].current_state_.adaptive_predicted_velocity_uncertainty));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &robot_structs_.hw_joint_struct_[i].current_state_.state_id));
    };
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &robot_structs_.current_state_.position_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &robot_structs_.current_state_.position_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &robot_structs_.current_state_.position_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &robot_structs_.current_state_.orientation_w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &robot_structs_.current_state_.orientation_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &robot_structs_.current_state_.orientation_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &robot_structs_.current_state_.orientation_z));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[0].name, &robot_structs_.mhe_data.time));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[1].name, &robot_structs_.mhe_data.t_step));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[2].name, &robot_structs_.mhe_data.je_ic_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[3].name, &robot_structs_.mhe_data.je_ic_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[4].name, &robot_structs_.mhe_data.je_ic_effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[5].name, &robot_structs_.mhe_data.jd_ic_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[6].name, &robot_structs_.mhe_data.jd_ic_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[7].name, &robot_structs_.mhe_data.jd_ic_effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[8].name, &robot_structs_.mhe_data.jc_ic_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[9].name, &robot_structs_.mhe_data.jc_ic_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[10].name, &robot_structs_.mhe_data.jc_ic_effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[11].name, &robot_structs_.mhe_data.jb_ic_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[12].name, &robot_structs_.mhe_data.jb_ic_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[13].name, &robot_structs_.mhe_data.jb_ic_effort));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[14].name, &robot_structs_.mhe_data.ja_ic_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[15].name, &robot_structs_.mhe_data.ja_ic_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[1].name, info_.gpios[1].state_interfaces[16].name, &robot_structs_.mhe_data.ja_ic_effort));
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  ReachSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &robot_structs_.hw_joint_struct_[i].command_state_.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &robot_structs_.hw_joint_struct_[i].command_state_.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &robot_structs_.hw_joint_struct_[i].command_state_.current));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &robot_structs_.hw_joint_struct_[i].command_state_.effort));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Activating... please wait...");
    try
    {
      driver_.setMode(alpha::driver::Mode::MODE_STANDBY, alpha::driver::DeviceId::kAllJoints);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), e.what()); // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "System successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ReachSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Deactivating... please wait...");
    try
    {
      driver_.setMode(alpha::driver::Mode::MODE_DISABLE, alpha::driver::DeviceId::kAllJoints);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), e.what()); // NOLINT
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Get access to the real-time states
    const std::lock_guard<std::mutex> lock(access_async_states_);

    if (rclcpp::ok())
    {
      rclcpp::spin_some(node_);
    }

    // RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "mhe size is %zu", latest_parameter_state_.data.size());

    // if (latest_parameter_state_.data.size() == 8)
    // {
    //   drag = latest_parameter_state_.data;
    // };

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ReachSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Send the commands for each joint
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_level_[i])
      {
      case mode_level_t::MODE_POSITION:
        if (!std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.position))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id);

          // Get the target position; if the command is for the jaws, then convert from m to mm
          const double target_position =
              static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id) == alpha::driver::DeviceId::kLinearJaws
                  ? robot_structs_.hw_joint_struct_[i].command_state_.position * 1000
                  : robot_structs_.hw_joint_struct_[i].command_state_.position;
          driver_.setPosition(target_position, target_device);
        }
        break;
      case mode_level_t::MODE_VELOCITY:
        if (!std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.velocity))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id);

          // Get the target velocity; if the command is for the jaws, then convert from m/s to mm/s
          const double target_velocity =
              static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id) == alpha::driver::DeviceId::kLinearJaws
                  ? robot_structs_.hw_joint_struct_[i].command_state_.velocity * 1000
                  : robot_structs_.hw_joint_struct_[i].command_state_.velocity;
          // RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "%d size is %f", static_cast<int>(target_device), target_velocity);

          driver_.setVelocity(target_velocity, target_device);
        }
        break;
      case mode_level_t::MODE_CURRENT:
        if (!std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.current))
        {
          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id);

          // enforce hard limit;
          const double enforced_target_current = robot_structs_.hw_joint_struct_[i].enforce_hard_limits(robot_structs_.hw_joint_struct_[i].command_state_.current);

          driver_.setCurrent(enforced_target_current, target_device);
          if (enforced_target_current == 0.0)
          {
            driver_.setVelocity(0.0, target_device); // incase of currents leak
          };
        }
        break;
      case mode_level_t::MODE_EFFORT:
        if (!std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.effort))
        {
          if (robot_structs_.hw_joint_struct_[i].command_state_.effort > 0)
          {
            T2C_arg = {DM(robot_structs_.hw_joint_struct_[i].actuator_Properties_.kt),
                       DM(robot_structs_.hw_joint_struct_[i].actuator_Properties_.forward_I_static),
                       DM(robot_structs_.hw_joint_struct_[i].command_state_.effort)};
          }
          else
          {
            T2C_arg = {DM(robot_structs_.hw_joint_struct_[i].actuator_Properties_.kt),
                       DM(robot_structs_.hw_joint_struct_[i].actuator_Properties_.backward_I_static),
                       DM(robot_structs_.hw_joint_struct_[i].command_state_.effort)};
          }

          std::vector<DM> currentMap = dynamics_service.torque2currentMap(T2C_arg);

          // Get the target device
          const auto target_device = static_cast<alpha::driver::DeviceId>(robot_structs_.hw_joint_struct_[i].device_id);

          // enforce hard limit;
          const double enforced_target_current = robot_structs_.hw_joint_struct_[i].enforce_hard_limits(currentMap.at(0).scalar());
          // RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "current from torque :::%f ", enforced_target_current);
          driver_.setCurrent(enforced_target_current, target_device);
          if (enforced_target_current == 0.0)
          {
            driver_.setVelocity(0.0, target_device); // incase of currents leak
          };
        }
        break;
      case mode_level_t::MODE_CARTESIAN:
        RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "endeffector commanding");
        break;
      case mode_level_t::MODE_STANDBY:
        // Handle standby mode if needed, or just break
        break;
      case mode_level_t::MODE_DISABLE:
        // Handle disable mode if needed, or just break
        break;
      default:
        // Existing code for default case...
        break;
      }
    }
    return hardware_interface::return_type::OK;
  }

  void ReachSystemMultiInterfaceHardware::updatePositionCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_struct_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float position;
    std::memcpy(&position, &packet.getData()[0], sizeof(position)); // NOLINT

    // Convert from mm to m if the message is from the jaws
    position = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? position / 1000 : position;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async position is %f", position);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_struct_ref.begin(), hw_joint_struct_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_struct_ref.end())
    {
      it->async_state_.position = position;
    }
  }

  void ReachSystemMultiInterfaceHardware::updateVelocityCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_struct_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float velocity;
    std::memcpy(&velocity, &packet.getData()[0], sizeof(velocity)); // NOLINT

    // Convert from mm/s to m/s if the message is from the jaws
    velocity = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? velocity / 1000 : velocity;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async velocity is %f", velocity);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_struct_ref.begin(), hw_joint_struct_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_struct_ref.end())
    {
      it->async_state_.velocity = velocity;
    }
  }

  void ReachSystemMultiInterfaceHardware::updateCurrentCb(const alpha::driver::Packet &packet, std::vector<Joint> &hw_joint_struct_ref)
  {
    if (packet.getData().size() != 4)
    {
      return;
    }

    float current;
    std::memcpy(&current, &packet.getData()[0], sizeof(current)); // NOLINT

    // Convert from mm/s to m/s if the message is from the jaws
    current = packet.getDeviceId() == alpha::driver::DeviceId::kLinearJaws ? current / 1000 : current;

    const std::lock_guard<std::mutex> lock(access_async_states_);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "async current is %f", current);

    auto deviceId = static_cast<uint8_t>(packet.getDeviceId()); // Cast the device ID to uint8_t

    auto it = std::find_if(hw_joint_struct_ref.begin(), hw_joint_struct_ref.end(),
                           [deviceId](const Joint &joint)
                           { return joint.device_id == deviceId; });

    if (it != hw_joint_struct_ref.end())
    {
      it->async_state_.current = current;
    }
  }

  void ReachSystemMultiInterfaceHardware::pollState(const int freq) const
  {
    while (running_.load())
    {
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_VELOCITY, alpha::driver::DeviceId::kRotateBase);

      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_POSITION, alpha::driver::DeviceId::kRotateBase);

      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kLinearJaws);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kRotateEndEffector);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kBendElbow);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kBendShoulder);
      driver_.request(alpha::driver::PacketId::PacketID_CURRENT, alpha::driver::DeviceId::kRotateBase);

      std::this_thread::sleep_for(std::chrono::seconds(1 / freq));
    }
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::ReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
