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
  constexpr auto DEFAULT_POSITION_PID_TOPIC = "~/position/reference";
  constexpr auto DEFAULT_VELOCITY_PID_TOPIC = "~/velocity/reference";
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
    dynamics_service.vehicle_dynamics = dynamics_service.load_casadi_fun("Vnext_Alloc", "libVnext.so");
    // dynamics_service.uvms_dynamics = dynamics_service.load_casadi_fun("uvms_stochastic_Alloc", "libUVMSnext.so");

    use_coupled_system = (info_.hardware_parameters["use_coupled_system"] == "True");
    use_thruster_command = (info_.hardware_parameters["use_thruster_command"] == "True");

    blue::dynamics::Vehicle::Pose_vel initial_state{0.0, 0.0, 2.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // // blue::dynamics::Vehicle rov{, initial_state};
    robot_structs_.hw_vehicle_struct_.set_vehicle_name("blue ROV heavy 0", initial_state);

    // robot_structs_.hw_vehicle_struct_.world_frame = info_.hardware_parameters["world_frame"];
    // robot_structs_.hw_vehicle_struct_.body_frame = info_.hardware_parameters["body_frame"];

    robot_structs_.hw_vehicle_struct_.thrustSizeAllocation(info_.joints.size());

    control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
      robot_structs_.hw_vehicle_struct_.hw_thrust_structs_.emplace_back(joint.name, defaultState);
      // RRBotSystemMultiInterface has exactly 3 state interfaces
      // and 3 command interfaces on each joint
      if (use_thruster_command)
      {
        if (joint.command_interfaces.size() != 2)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
              "Thruster '%s' has %zu command interfaces. 2 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      if (joint.state_interfaces.size() != 4)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "Thruster '%s'has %zu state interfaces. 4 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
    };

    for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
    {
      if (gpio.state_interfaces.size() != 19)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
            "GPIO '%s'has %zu state interfaces. 19 expected.", gpio.name.c_str(),
            gpio.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!use_thruster_command)
      {
        if (gpio.command_interfaces.size() != 19)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
              "GPIO '%s'has %zu command interfaces. 19 expected.", gpio.name.c_str(),
              gpio.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }
      };
    };

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers, and ROS subscribers ...
    try
    {
      auto node_topics_interface = rclcpp::Node("VehicleSystemMultiInterfaceHardware");

      // topics QoS
      auto subscribers_qos = rclcpp::SystemDefaultsQoS();
      subscribers_qos.keep_last(1);
      subscribers_qos.best_effort();

      // position Reference Subscriber
      position_ref_subscriber_ = rclcpp::create_subscription<RefType>(node_topics_interface,
                                                                      DEFAULT_POSITION_PID_TOPIC, subscribers_qos,
                                                                      [this](const RefType::SharedPtr msg)
                                                                      { rt_position_ptr_.writeFromNonRT(msg); });

      // velocity Reference Subscriber
      velocity_ref_subscriber_ = rclcpp::create_subscription<RefType>(node_topics_interface,
                                                                      DEFAULT_VELOCITY_PID_TOPIC, subscribers_qos,
                                                                      [this](const RefType::SharedPtr msg)
                                                                      { rt_velocity_ptr_.writeFromNonRT(msg); });

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
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.effort));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &robot_structs_.hw_vehicle_struct_.current_state_.position_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &robot_structs_.hw_vehicle_struct_.current_state_.position_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &robot_structs_.hw_vehicle_struct_.current_state_.position_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &robot_structs_.hw_vehicle_struct_.current_state_.orientation_w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &robot_structs_.hw_vehicle_struct_.current_state_.orientation_x));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &robot_structs_.hw_vehicle_struct_.current_state_.orientation_y));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &robot_structs_.hw_vehicle_struct_.current_state_.orientation_z));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[7].name, &robot_structs_.hw_vehicle_struct_.current_state_.u));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[8].name, &robot_structs_.hw_vehicle_struct_.current_state_.v));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[9].name, &robot_structs_.hw_vehicle_struct_.current_state_.w));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[10].name, &robot_structs_.hw_vehicle_struct_.current_state_.p));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[11].name, &robot_structs_.hw_vehicle_struct_.current_state_.q));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[12].name, &robot_structs_.hw_vehicle_struct_.current_state_.r));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[13].name, &robot_structs_.hw_vehicle_struct_.current_state_.Fx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[14].name, &robot_structs_.hw_vehicle_struct_.current_state_.Fy));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[15].name, &robot_structs_.hw_vehicle_struct_.current_state_.Fz));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[16].name, &robot_structs_.hw_vehicle_struct_.current_state_.Tx));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[17].name, &robot_structs_.hw_vehicle_struct_.current_state_.Ty));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios[0].name, info_.gpios[0].state_interfaces[18].name, &robot_structs_.hw_vehicle_struct_.current_state_.Tz));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  VehicleSystemMultiInterfaceHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    if (use_thruster_command)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.effort));
      }
    }
    else
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &robot_structs_.hw_vehicle_struct_.command_state_.position_x));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &robot_structs_.hw_vehicle_struct_.command_state_.position_y));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &robot_structs_.hw_vehicle_struct_.command_state_.position_z));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &robot_structs_.hw_vehicle_struct_.command_state_.orientation_w));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &robot_structs_.hw_vehicle_struct_.command_state_.orientation_x));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &robot_structs_.hw_vehicle_struct_.command_state_.orientation_y));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &robot_structs_.hw_vehicle_struct_.command_state_.orientation_z));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[7].name, &robot_structs_.hw_vehicle_struct_.command_state_.u));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[8].name, &robot_structs_.hw_vehicle_struct_.command_state_.v));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[9].name, &robot_structs_.hw_vehicle_struct_.command_state_.w));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[10].name, &robot_structs_.hw_vehicle_struct_.command_state_.p));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[11].name, &robot_structs_.hw_vehicle_struct_.command_state_.q));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[12].name, &robot_structs_.hw_vehicle_struct_.command_state_.r));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[13].name, &robot_structs_.hw_vehicle_struct_.command_state_.Fx));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[14].name, &robot_structs_.hw_vehicle_struct_.command_state_.Fy));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[15].name, &robot_structs_.hw_vehicle_struct_.command_state_.Fz));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[16].name, &robot_structs_.hw_vehicle_struct_.command_state_.Tx));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[17].name, &robot_structs_.hw_vehicle_struct_.command_state_.Ty));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.gpios[0].name, info_.gpios[0].command_interfaces[18].name, &robot_structs_.hw_vehicle_struct_.command_state_.Tz));
    }
    return command_interfaces;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    // Prepare for new command modes
    std::vector<mode_level_t> new_modes = {};
    if (use_thruster_command)
    {
      for (std::string key : start_interfaces)
      {
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
          {
            new_modes.push_back(mode_level_t::MODE_CURRENT);
          }
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
          {
            new_modes.push_back(mode_level_t::MODE_CURRENT);
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

      // Example criteria: All joints must be given new command mode at the same time
      if (new_modes.size() != info_.joints.size())
      {
        return hardware_interface::return_type::ERROR;
      }
      // Example criteria: All joints must have the same command mode
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
            robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current = 0;
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
            else if (info_.gpios[0].command_interfaces[j].name.find("velocity") != std::string::npos)
            {
              control_level_.push_back(mode_level_t::MODE_VELOCITY);
            }
            else if (info_.gpios[0].command_interfaces[j].name.find("force") != std::string::npos ||
                     info_.gpios[0].command_interfaces[j].name.find("torque") != std::string::npos)
            {
              control_level_.push_back(mode_level_t::MODE_EFFORT_GENERALIZED);
            }
          }
        }
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn VehicleSystemMultiInterfaceHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Activating... please wait...");

    // reset position buffer if a ref came through callback when controller was inactive
    rt_position_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);

    // reset velocity buffer if a ref came through callback when controller was inactive
    rt_velocity_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.position = 0.0;
      }
      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.velocity = 0.0;
      }
      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.current = 0.0;
      }
      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].current_state_.acceleration = 0.0;
      }

      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.current = 0.0;
      }
      if (std::isnan(robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.effort))
      {
        robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[i].command_state_.effort = 0.0;
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
    // reset position buffer if a ref came through callback when controller was inactive
    rt_position_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);
    // reset velocity buffer if a ref came through callback when controller was inactive
    rt_velocity_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);
    RCLCPP_INFO(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_DEBUG(
    //     rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"),
    //     "Got commands: %.5f,  %.5f, %.5f, %.5f, %.5f,  %.5f, %.5f, %.5f ",
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[0].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[1].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[2].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[3].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[4].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[5].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[6].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[7].command_state_.effort);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type VehicleSystemMultiInterfaceHardware::write(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    double delta_seconds = period.seconds();

    // auto position_reference_state = rt_position_ptr_.readFromRT();

    // // no positon reference received yet
    // if (!position_reference_state || !(*position_reference_state))
    // {
    //   return hardware_interface::return_type::OK;
    // }

    // auto vel_reference_state = rt_velocity_ptr_.readFromRT();

    // // no velocity reference received yet
    // if (!vel_reference_state || !(*vel_reference_state))
    // {
    //   return hardware_interface::return_type::OK;
    // }
    if (!use_coupled_system)
    {

      std::vector<double> x0 = {
          robot_structs_.hw_vehicle_struct_.current_state_.position_x,
          robot_structs_.hw_vehicle_struct_.current_state_.position_y,
          robot_structs_.hw_vehicle_struct_.current_state_.position_z,
          robot_structs_.hw_vehicle_struct_.current_state_.orientation_w,
          robot_structs_.hw_vehicle_struct_.current_state_.orientation_x,
          robot_structs_.hw_vehicle_struct_.current_state_.orientation_y,
          robot_structs_.hw_vehicle_struct_.current_state_.orientation_z,
          robot_structs_.hw_vehicle_struct_.current_state_.u,
          robot_structs_.hw_vehicle_struct_.current_state_.v,
          robot_structs_.hw_vehicle_struct_.current_state_.w,
          robot_structs_.hw_vehicle_struct_.current_state_.p,
          robot_structs_.hw_vehicle_struct_.current_state_.q,
          robot_structs_.hw_vehicle_struct_.current_state_.r};

      std::vector<double> u0 = {robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[0].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[1].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[2].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[3].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[4].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[5].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[6].command_state_.effort,
                                robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[7].command_state_.effort};
      std::vector<double> vc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::vector<DM> dynamic_arg = {DM(x0), DM(u0), DM(delta_seconds), DM(vc)};
      RCLCPP_DEBUG(rclcpp::get_logger("VehicleSystemMultiInterfaceHardware"), "Got states: %.5f second interval, %.5f,  %.5f, %.5f, %.5f, %.5f,  %.5f, %.5f, %.5f ",
                   delta_seconds,
                   robot_structs_.hw_vehicle_struct_.current_state_.position_x,
                   robot_structs_.hw_vehicle_struct_.current_state_.position_y,
                   robot_structs_.hw_vehicle_struct_.current_state_.position_z,
                   robot_structs_.hw_vehicle_struct_.current_state_.orientation_w,
                   robot_structs_.hw_vehicle_struct_.current_state_.orientation_x,
                   robot_structs_.hw_vehicle_struct_.current_state_.orientation_y,
                   robot_structs_.hw_vehicle_struct_.current_state_.orientation_z,
                   robot_structs_.hw_vehicle_struct_.current_state_.u);

      std::vector<DM> dynamic_response = dynamics_service.vehicle_dynamics(dynamic_arg);
      forward_dynamics_res = std::vector<double>(dynamic_response.at(0));

      robot_structs_.hw_vehicle_struct_.current_state_.position_x = forward_dynamics_res[0];
      robot_structs_.hw_vehicle_struct_.current_state_.position_y = forward_dynamics_res[1];
      robot_structs_.hw_vehicle_struct_.current_state_.position_z = forward_dynamics_res[2];
      robot_structs_.hw_vehicle_struct_.current_state_.orientation_w = forward_dynamics_res[3];
      robot_structs_.hw_vehicle_struct_.current_state_.orientation_x = forward_dynamics_res[4];
      robot_structs_.hw_vehicle_struct_.current_state_.orientation_y = forward_dynamics_res[5];
      robot_structs_.hw_vehicle_struct_.current_state_.orientation_z = forward_dynamics_res[6];
      robot_structs_.hw_vehicle_struct_.current_state_.u = forward_dynamics_res[7];
      robot_structs_.hw_vehicle_struct_.current_state_.v = forward_dynamics_res[8];
      robot_structs_.hw_vehicle_struct_.current_state_.w = forward_dynamics_res[9];
      robot_structs_.hw_vehicle_struct_.current_state_.p = forward_dynamics_res[10];
      robot_structs_.hw_vehicle_struct_.current_state_.q = forward_dynamics_res[11];
      robot_structs_.hw_vehicle_struct_.current_state_.r = forward_dynamics_res[12];
    }

    if (realtime_odometry_transform_publisher_->trylock())
    {
      // original pose in NED
      // RBIZ USES NWU
      tf2::Quaternion q_orig, q_rot, q_new;

      q_orig.setW(robot_structs_.hw_vehicle_struct_.current_state_.orientation_w);
      q_orig.setX(robot_structs_.hw_vehicle_struct_.current_state_.orientation_x);
      q_orig.setY(robot_structs_.hw_vehicle_struct_.current_state_.orientation_y);
      q_orig.setZ(robot_structs_.hw_vehicle_struct_.current_state_.orientation_z);
      // Rotate the previous pose by 180* about X
      // q_rot.setRPY(3.14159, 0.0, 0.0);

      // Rotate the previous pose by 0* about X
      q_rot.setRPY(0.0, 0.0, 0.0);
      q_new = q_rot * q_orig;
      q_new.normalize();

      auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = robot_structs_.hw_vehicle_struct_.current_state_.position_x;
      transform.transform.translation.y = -robot_structs_.hw_vehicle_struct_.current_state_.position_y;
      transform.transform.translation.z = -robot_structs_.hw_vehicle_struct_.current_state_.position_z;

      transform.transform.rotation.x = q_new.x();
      transform.transform.rotation.y = q_new.y();
      transform.transform.rotation.z = q_new.z();
      transform.transform.rotation.w = q_new.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::VehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
