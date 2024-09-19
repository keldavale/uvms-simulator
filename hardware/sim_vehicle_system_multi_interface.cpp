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

#include "ros2_control_blue_reach_5/sim_vehicle_system_multi_interface.hpp"

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
  constexpr auto DEFAULT_DVL_TOPIC = "/uvms/dvl";
} // namespace

namespace ros2_control_blue_reach_5
{
  hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_init(
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
    RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
    // Use CasADi's "external" to load the compiled dynamics functions
    // dynamics_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");
    // dynamics_service.vehicle_dynamics = dynamics_service.load_casadi_fun("Vnext_Alloc", "libVnext.so");
    hw_vehicle_struct_.resize(1);

    blue::dynamics::Vehicle::Pose_vel initial_state{
        0.0, 0.0, 2.5,      // Position: x, y, z
        1.0, 0.0, 0.0, 0.0, // Orientation: qw, qx, qy, qz
        0.0, 0.0, 0.0,      // Linear velocities: vx, vy, vz
        0.0, 0.0, 0.0,      // Angular velocities: wx, wy, wz
        0.0, 0.0, 0.0,      // Forces: Fx, Fy, Fz
        0.0, 0.0, 0.0       // Torques: Tx, Ty, Tz
    };

    hw_vehicle_struct_[0].set_vehicle_name("blue ROV heavy 0", initial_state);

    hw_vehicle_struct_[0].thrustSizeAllocation(info_.joints.size());

    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
      hw_vehicle_struct_[0].hw_thrust_structs_.emplace_back(joint.name, defaultState);
      // RRBotSystemMultiInterface has exactly 4 joint state interfaces
      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "Thruster '%s'has %zu state interfaces. 4 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      };
    };

    for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
    {
      // RRBotSystemMultiInterface has exactly 25 gpio state interfaces
      if (gpio.state_interfaces.size() != 25)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu state interfaces. 25 expected.", gpio.name.c_str(),
            gpio.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // RRBotSystemMultiInterface has exactly 25 gpio command interfaces
      if (gpio.command_interfaces.size() != 25)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu command interfaces. 25 expected.", gpio.name.c_str(),
            gpio.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    };

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers ...
    try
    {
      auto node_topics_interface = rclcpp::Node("SimVehicleSystemMultiInterfaceHardware");

      // tf publisher
      odometry_transform_publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(node_topics_interface,
                                                                                         DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_odometry_transform_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
              odometry_transform_publisher_);

      uv_dvl_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::Twist>(node_topics_interface,
                                                                              DEFAULT_DVL_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_uv_dvl_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
              uv_dvl_publisher_);

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
        rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "configure successful");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  SimVehicleSystemMultiInterfaceHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.current));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.effort));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &hw_vehicle_struct_[0].current_state_.position_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &hw_vehicle_struct_[0].current_state_.position_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &hw_vehicle_struct_[0].current_state_.position_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &hw_vehicle_struct_[0].current_state_.orientation_w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &hw_vehicle_struct_[0].current_state_.orientation_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &hw_vehicle_struct_[0].current_state_.orientation_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &hw_vehicle_struct_[0].current_state_.orientation_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[7].name, &hw_vehicle_struct_[0].current_state_.u));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[8].name, &hw_vehicle_struct_[0].current_state_.v));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[9].name, &hw_vehicle_struct_[0].current_state_.w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[10].name, &hw_vehicle_struct_[0].current_state_.p));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[11].name, &hw_vehicle_struct_[0].current_state_.q));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[12].name, &hw_vehicle_struct_[0].current_state_.r));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[13].name, &hw_vehicle_struct_[0].current_state_.du));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[14].name, &hw_vehicle_struct_[0].current_state_.dv));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[15].name, &hw_vehicle_struct_[0].current_state_.dw));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[16].name, &hw_vehicle_struct_[0].current_state_.dp));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[17].name, &hw_vehicle_struct_[0].current_state_.dq));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[18].name, &hw_vehicle_struct_[0].current_state_.dr));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[19].name, &hw_vehicle_struct_[0].current_state_.Fx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[20].name, &hw_vehicle_struct_[0].current_state_.Fy));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[21].name, &hw_vehicle_struct_[0].current_state_.Fz));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[22].name, &hw_vehicle_struct_[0].current_state_.Tx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[23].name, &hw_vehicle_struct_[0].current_state_.Ty));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[24].name, &hw_vehicle_struct_[0].current_state_.Tz));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  SimVehicleSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &hw_vehicle_struct_[0].command_state_.position_x));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &hw_vehicle_struct_[0].command_state_.position_y));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &hw_vehicle_struct_[0].command_state_.position_z));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &hw_vehicle_struct_[0].command_state_.orientation_w));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &hw_vehicle_struct_[0].command_state_.orientation_x));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &hw_vehicle_struct_[0].command_state_.orientation_y));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &hw_vehicle_struct_[0].command_state_.orientation_z));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[7].name, &hw_vehicle_struct_[0].command_state_.u));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[8].name, &hw_vehicle_struct_[0].command_state_.v));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[9].name, &hw_vehicle_struct_[0].command_state_.w));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[10].name, &hw_vehicle_struct_[0].command_state_.p));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[11].name, &hw_vehicle_struct_[0].command_state_.q));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[12].name, &hw_vehicle_struct_[0].command_state_.r));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[13].name, &hw_vehicle_struct_[0].command_state_.du));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[14].name, &hw_vehicle_struct_[0].command_state_.dv));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[15].name, &hw_vehicle_struct_[0].command_state_.dw));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[16].name, &hw_vehicle_struct_[0].command_state_.dp));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[17].name, &hw_vehicle_struct_[0].command_state_.dq));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[18].name, &hw_vehicle_struct_[0].command_state_.dr));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[19].name, &hw_vehicle_struct_[0].command_state_.Fx));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[20].name, &hw_vehicle_struct_[0].command_state_.Fy));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[21].name, &hw_vehicle_struct_[0].command_state_.Fz));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[22].name, &hw_vehicle_struct_[0].command_state_.Tx));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[23].name, &hw_vehicle_struct_[0].command_state_.Ty));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[0].name, info_.gpios[0].command_interfaces[24].name, &hw_vehicle_struct_[0].command_state_.Tz));

    return command_interfaces;
  }

  hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
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
          if (info_.gpios[0].command_interfaces[j].name.find("orientation") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_POSITION);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("velocity") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_VELOCITY);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("acceleration") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_ACCELERATION);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("force") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_EFFORT_GENERALIZED);
          }
          else if (info_.gpios[0].command_interfaces[j].name.find("torque") != std::string::npos)
          {
            new_modes.push_back(mode_level_t::MODE_EFFORT_GENERALIZED);
          }
        }
      }
    };

    //  criteria: All joints must be given new command mode at the same time
    if (new_modes.size() != 25)
    {
      return hardware_interface::return_type::ERROR;
    };
    RCLCPP_INFO(
        rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Command Mode Switch successful");
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Activating... please wait...");

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.position))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.position = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.velocity))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.velocity = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.current))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.current = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.acceleration))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.acceleration = 0.0;
      }

      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].command_state_.current))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].command_state_.current = 0.0;
      }
      if (std::isnan(hw_vehicle_struct_[0].hw_thrust_structs_[i].command_state_.effort))
      {
        hw_vehicle_struct_[0].hw_thrust_structs_[i].command_state_.effort = 0.0;
      }
      control_level_[i] = mode_level_t::MODE_DISABLE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "System successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Deactivating... please wait...");
    RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_DEBUG(
    //     rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
    //     "Got commands: %.5f,  %.5f, %.5f, %.5f, %.5f,  %.5f, %.5f, %.5f ",
    //     hw_vehicle_struct_[0].hw_thrust_structs_[0].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[1].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[2].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[3].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[4].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[5].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[6].command_state_.effort,
    //     hw_vehicle_struct_[0].hw_thrust_structs_[7].command_state_.effort);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::write(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(
    //     rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
    //     "Got commands: %f,  %f, %f, %f, %f,  %f",
    //     hw_vehicle_struct_[0].command_state_.Fx,
    //     hw_vehicle_struct_[0].command_state_.Fy,
    //     hw_vehicle_struct_[0].command_state_.Fz,
    //     hw_vehicle_struct_[0].command_state_.Tx,
    //     hw_vehicle_struct_[0].command_state_.Ty,
    //     hw_vehicle_struct_[0].command_state_.Tz);

    hw_vehicle_struct_[0].current_state_.position_x = hw_vehicle_struct_[0].command_state_.position_x;
    hw_vehicle_struct_[0].current_state_.position_y = hw_vehicle_struct_[0].command_state_.position_y;
    hw_vehicle_struct_[0].current_state_.position_z = hw_vehicle_struct_[0].command_state_.position_z;
    hw_vehicle_struct_[0].current_state_.orientation_w = hw_vehicle_struct_[0].command_state_.orientation_w;
    hw_vehicle_struct_[0].current_state_.orientation_x = hw_vehicle_struct_[0].command_state_.orientation_x;
    hw_vehicle_struct_[0].current_state_.orientation_y = hw_vehicle_struct_[0].command_state_.orientation_y;
    hw_vehicle_struct_[0].current_state_.orientation_z = hw_vehicle_struct_[0].command_state_.orientation_z;

    hw_vehicle_struct_[0].current_state_.u = hw_vehicle_struct_[0].command_state_.u;
    hw_vehicle_struct_[0].current_state_.v = hw_vehicle_struct_[0].command_state_.v;
    hw_vehicle_struct_[0].current_state_.w = hw_vehicle_struct_[0].command_state_.w;
    hw_vehicle_struct_[0].current_state_.p = hw_vehicle_struct_[0].command_state_.p;
    hw_vehicle_struct_[0].current_state_.q = hw_vehicle_struct_[0].command_state_.q;
    hw_vehicle_struct_[0].current_state_.r = hw_vehicle_struct_[0].command_state_.r;

    hw_vehicle_struct_[0].current_state_.Fx = hw_vehicle_struct_[0].command_state_.Fx;
    hw_vehicle_struct_[0].current_state_.Fy = hw_vehicle_struct_[0].command_state_.Fy;
    hw_vehicle_struct_[0].current_state_.Fz = -hw_vehicle_struct_[0].command_state_.Fz;
    hw_vehicle_struct_[0].current_state_.Tx = hw_vehicle_struct_[0].command_state_.Tx;
    hw_vehicle_struct_[0].current_state_.Ty = hw_vehicle_struct_[0].command_state_.Ty;
    hw_vehicle_struct_[0].current_state_.Tz = hw_vehicle_struct_[0].command_state_.Tz;

    if (realtime_odometry_transform_publisher_ && realtime_odometry_transform_publisher_->trylock())
    {
      // original pose in NED
      // RVIZ USES NWU
      tf2::Quaternion q_orig, q_rot, q_new;

      q_orig.setW(hw_vehicle_struct_[0].current_state_.orientation_w);
      q_orig.setX(hw_vehicle_struct_[0].current_state_.orientation_x);
      q_orig.setY(hw_vehicle_struct_[0].current_state_.orientation_y);
      q_orig.setZ(hw_vehicle_struct_[0].current_state_.orientation_z);
      // Rotate the previous pose by 180* about X
      // q_rot.setRPY(3.14159, 0.0, 0.0);

      // Rotate the previous pose by 0* about X
      q_rot.setRPY(0.0, 0.0, 0.0);
      q_new = q_rot * q_orig;
      q_new.normalize();

      auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = hw_vehicle_struct_[0].current_state_.position_x;
      transform.transform.translation.y = hw_vehicle_struct_[0].current_state_.position_y;
      transform.transform.translation.z = -hw_vehicle_struct_[0].current_state_.position_z;

      transform.transform.rotation.x = q_new.x();
      transform.transform.rotation.y = q_new.y();
      transform.transform.rotation.z = q_new.z();
      transform.transform.rotation.w = q_new.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    };

    if (realtime_uv_dvl_publisher_ && realtime_uv_dvl_publisher_->trylock())
    {
      auto &dvl_message = realtime_uv_dvl_publisher_->msg_;
      dvl_message.linear.x = hw_vehicle_struct_[0].current_state_.u;
      dvl_message.linear.y = hw_vehicle_struct_[0].current_state_.v;
      dvl_message.linear.z = hw_vehicle_struct_[0].current_state_.w;
      dvl_message.angular.x = hw_vehicle_struct_[0].current_state_.p;
      dvl_message.angular.y = hw_vehicle_struct_[0].current_state_.q;
      dvl_message.angular.z = hw_vehicle_struct_[0].current_state_.r;
      realtime_uv_dvl_publisher_->unlockAndPublish();
    };

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimVehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
