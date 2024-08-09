#include "ros2_control_blue_reach_5/rrbot_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{
  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_init(
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
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "CasADi computer from manipulator system: %s", casadi_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
    // Use CasADi's "external" to load the compiled dynamics functions
    dynamics_service.usage_cplusplus_checks("test", "libtest.so", "rrbot system");

    use_coupled_system = (info_.hardware_parameters["use_coupled_system"] == "True");
    endeffector_control = (info_.hardware_parameters["endeffector_control"] == "True");

    robot_structs_.hw_joint_struct_.reserve(info_.joints.size());
    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      std::string device_id_value = joint.parameters.at("device_id");
      double default_position = stod(joint.parameters.at("home"));

      uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

      RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
      RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

      Joint::State initialState{default_position, 0.0, 0.0};
      robot_structs_.hw_joint_struct_.emplace_back(joint.name, device_id, initialState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (!endeffector_control)
      {
        if (joint.command_interfaces.size() != 4)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
              "Joint '%s' has %zu command interfaces. 4 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }
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
    for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
    {
      if (endeffector_control)
      {
        if (gpio.command_interfaces.size() != 7)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
              "GPIO '%s'has %zu command interfaces. 7 expected.", gpio.name.c_str(),
              gpio.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      if (gpio.state_interfaces.size() != 7)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ReachSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu state interfaces. 7 expected.", gpio.name.c_str(),
            gpio.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Successfully configured the RRBotSystemMultiInterfaceHardware system interface for serial communication!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO( // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  RRBotSystemMultiInterfaceHardware::export_state_interfaces()
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
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  RRBotSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    if (endeffector_control)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &robot_structs_.command_state_.position_x));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &robot_structs_.command_state_.position_y));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &robot_structs_.command_state_.position_z));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &robot_structs_.command_state_.orientation_w));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &robot_structs_.command_state_.orientation_x));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &robot_structs_.command_state_.orientation_y));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &robot_structs_.command_state_.orientation_z));
    }
    else
    {
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
    }
    return command_interfaces;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> & /*stop_interfaces*/)
  {
    // Prepare for new command modes
    std::vector<mode_level_t> new_modes = {};
    if (!endeffector_control)
    {
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
      }
      control_level_ = new_modes;
    }
    else
    {
      for (std::string key : start_interfaces)
      {
        for (std::size_t j = 0; j < info_.gpios[0].command_interfaces.size(); j++)
        {
          std::string full_name = info_.gpios[0].name + "/" + info_.gpios[0].command_interfaces[j].name;
          if (key == full_name)
          {
            if (info_.gpios[0].command_interfaces[j].name.find("position") != std::string::npos ||
                info_.gpios[0].command_interfaces[j].name.find("orientation") != std::string::npos)
            {
              control_level_.push_back(mode_level_t::MODE_POSITION);
            }
          }
        }
      }
    };
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Activating... please wait...");

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

      if (std::isnan(robot_structs_.hw_joint_struct_[i].current_state_.position) || robot_structs_.hw_joint_struct_[i].current_state_.position == 0)
      {
        robot_structs_.hw_joint_struct_[i].current_state_.position = robot_structs_.hw_joint_struct_[i].default_state_.position;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].current_state_.velocity))
      {
        robot_structs_.hw_joint_struct_[i].current_state_.velocity = 0;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].current_state_.current))
      {
        robot_structs_.hw_joint_struct_[i].current_state_.current = 0;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].current_state_.acceleration))
      {
        robot_structs_.hw_joint_struct_[i].current_state_.acceleration = 0;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.position))
      {
        robot_structs_.hw_joint_struct_[i].command_state_.position = 0;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.velocity))
      {
        robot_structs_.hw_joint_struct_[i].command_state_.velocity = 0;
      }
      if (std::isnan(robot_structs_.hw_joint_struct_[i].command_state_.current))
      {
        robot_structs_.hw_joint_struct_[i].command_state_.current = 0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Deactivating... please wait...");

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RRBotSystemMultiInterfaceHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::RRBotSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
