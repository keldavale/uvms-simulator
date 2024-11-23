// Copyright 2024, Edward Morgan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ros2_control_blue_reach_5/vehicle_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace
{
  constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace ros2_control_blue_reach_5
{
  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_init(
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
    RCLCPP_INFO(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
    // Use CasADi's "external" to load the compiled dynamics functions
    dynamics_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");

    blue::dynamics::Vehicle::Pose_vel initial_state{0.0, 0.0, 2.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    hw_vehicle_struct_.set_vehicle_name("blue ROV heavy 0", initial_state);

    // hw_vehicle_struct_.world_frame = info_.hardware_parameters["world_frame"];
    // hw_vehicle_struct_.body_frame = info_.hardware_parameters["body_frame"];

    hw_vehicle_struct_.thrustSizeAllocation(info_.joints.size());

    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
      hw_vehicle_struct_.hw_thrust_structs_.emplace_back(joint.name, defaultState);
      // RRBotSystemMultiInterface has exactly 4 joint state interfaces
      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s'has %zu state interfaces. 4 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      };
    };

    for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
    {
      // RRBotSystemMultiInterface has exactly 19 gpio state interfaces
      if (gpio.state_interfaces.size() != 19)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu state interfaces. 19 expected.", gpio.name.c_str(),
            gpio.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // RRBotSystemMultiInterface has exactly 19 gpio command interfaces
      if (gpio.command_interfaces.size() != 19)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu command interfaces. 19 expected.", gpio.name.c_str(),
            gpio.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    };

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers ...
    try
    {
      auto node_topics_interface = rclcpp::Node("VehicleSystemMultiInterfaceHardware");

      // tf publisher
      odometry_transform_publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node_topics_interface,
                                                                                         DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

      realtime_odometry_transform_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
              odometry_transform_publisher_);

      auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
      odometry_transform_message.transforms.resize(1);
      odometry_transform_message.transforms.front().header.frame_id = "world";
      odometry_transform_message.transforms.front().child_frame_id = "base_link";
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "configure successful");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  VehicleSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.effort));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &hw_vehicle_struct_.current_state_.position_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &hw_vehicle_struct_.current_state_.position_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &hw_vehicle_struct_.current_state_.position_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &hw_vehicle_struct_.current_state_.orientation_w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &hw_vehicle_struct_.current_state_.orientation_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &hw_vehicle_struct_.current_state_.orientation_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &hw_vehicle_struct_.current_state_.orientation_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[7].name, &hw_vehicle_struct_.current_state_.u));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[8].name, &hw_vehicle_struct_.current_state_.v));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[9].name, &hw_vehicle_struct_.current_state_.w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[10].name, &hw_vehicle_struct_.current_state_.p));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[11].name, &hw_vehicle_struct_.current_state_.q));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[12].name, &hw_vehicle_struct_.current_state_.r));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[13].name, &hw_vehicle_struct_.current_state_.Fx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[14].name, &hw_vehicle_struct_.current_state_.Fy));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[15].name, &hw_vehicle_struct_.current_state_.Fz));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[16].name, &hw_vehicle_struct_.current_state_.Tx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[17].name, &hw_vehicle_struct_.current_state_.Ty));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[18].name, &hw_vehicle_struct_.current_state_.Tz));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  VehicleSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &hw_vehicle_struct_.command_state_.position_x));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &hw_vehicle_struct_.command_state_.position_y));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &hw_vehicle_struct_.command_state_.position_z));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &hw_vehicle_struct_.command_state_.orientation_w));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &hw_vehicle_struct_.command_state_.orientation_x));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &hw_vehicle_struct_.command_state_.orientation_y));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &hw_vehicle_struct_.command_state_.orientation_z));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[7].name, &hw_vehicle_struct_.command_state_.u));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[8].name, &hw_vehicle_struct_.command_state_.v));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[9].name, &hw_vehicle_struct_.command_state_.w));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[10].name, &hw_vehicle_struct_.command_state_.p));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[11].name, &hw_vehicle_struct_.command_state_.q));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[12].name, &hw_vehicle_struct_.command_state_.r));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[13].name, &hw_vehicle_struct_.command_state_.Fx));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[14].name, &hw_vehicle_struct_.command_state_.Fy));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[15].name, &hw_vehicle_struct_.command_state_.Fz));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[16].name, &hw_vehicle_struct_.command_state_.Tx));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[17].name, &hw_vehicle_struct_.command_state_.Ty));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[18].name, &hw_vehicle_struct_.command_state_.Tz));

    return command_interfaces;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & /*stop_interfaces*/)
  {
    // Prepare for new command modes
    std::vector<mode_level_t> new_modes = {};

    for (std::string key : start_interfaces)
    {
      for (std::size_t j = 0; j < info_.gpios[0].command_interfaces.size(); j++)
      {
        std::string full_name = info_.gpios[0].name + "/" + info_.gpios[0].command_interfaces[j].name;
        if (key == full_name)
        {
          if (info_.gpios[0].command_interfaces[j].name.find("position") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_POSITION);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("velocity") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_VELOCITY);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("effort") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_EFFORT_GENERALIZED);
          }
        }
      }
    };

    //  criteria: All joints must be given new command mode at the same time
    // if (new_modes.size() != info_.gpios[0].command_interfaces.size()*3)
    // {
    //   return hardware_interface::return_type::ERROR;
    // };

    //  criteria: All joints must have the same command mode
    // if (!std::all_of(
    //         new_modes.begin() + 1, new_modes.end(),
    //         [&](mode_level_t mode)
    //         { return mode == new_modes[0]; }))
    // {
    //   return hardware_interface::return_type::ERROR;
    // };

    control_level_ = new_modes;
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Command Mode Switch successful");
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Activating... please wait...");

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration = 0.0;
      }

      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.effort))
      {
        hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.effort = 0.0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "System successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Deactivating... please wait...");
    RCLCPP_INFO(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::write(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    double delta_seconds = period.seconds();

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::VehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
