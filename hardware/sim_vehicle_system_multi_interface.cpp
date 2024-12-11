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
#include <random>

using namespace casadi;

namespace
{
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
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
        // Access the name from the HardwareInfo
        system_name = info.name;
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        cfg_.frame_id = info_.hardware_parameters["frame_id"];
        cfg_.child_frame_id = info_.hardware_parameters["child_frame_id"];
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************frame id: %s", cfg_.frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************child frame id: %s", cfg_.child_frame_id.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled dynamics functions
        dynamics_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");

        hw_vehicle_struct_.resize(1);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(-5.0, 5.0);
        std::uniform_real_distribution<> dis_y(-5.0, 5.0);
        std::uniform_real_distribution<> dis_z(3.0, 7.0);

        blue::dynamics::Vehicle::Pose_vel initial_state{
            dis_x(gen), dis_y(gen), 5.0, // Randomized position: x, y, z
            1.0, 0.0, 0.0, 0.0,          // Orientation: qw, qx, qy, qz
            0.0, 0.0, 0.0,               // Linear velocities: vx, vy, vz
            0.0, 0.0, 0.0,               // Angular velocities: wx, wy, wz
            0.0, 0.0, 0.0,               // Forces: Fx, Fy, Fz
            0.0, 0.0, 0.0                // Torques: Tx, Ty, Tz
        };

        hw_vehicle_struct_[0].set_vehicle_name("blue ROV heavy 0", initial_state);

        hw_vehicle_struct_[0].thrustSizeAllocation(info_.joints.size());

        control_level_.resize(info_.joints.size(), mode_level_t::MODE_DISABLE);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
            hw_vehicle_struct_[0].hw_thrust_structs_.emplace_back(joint.name, defaultState);
            // RRBotSystemMultiInterface has exactly 6 joint state interfaces
            if (joint.state_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu state interfaces. 6 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };
        };

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // RRBotSystemMultiInterface has exactly 25 gpio state interfaces
            if (gpio.state_interfaces.size() != 27)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 27 expected.", gpio.name.c_str(),
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
            // Create a shared pointer to the node
            auto node_topics_interface = std::make_shared<rclcpp::Node>(system_name);

            // Retrieve the node name and log it
            RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                        "publisher node name: %s", node_topics_interface->get_name());

            // tf publisher
            transform_publisher_ = rclcpp::create_publisher<tf>(node_topics_interface, DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(
                    transform_publisher_);

            auto &transform_message = realtime_transform_publisher_->msg_;
            transform_message.transforms.resize(1);
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

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.sim_period));
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

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[25].name, &hw_vehicle_struct_[0].current_state_.sim_time));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[26].name, &hw_vehicle_struct_[0].current_state_.sim_period));
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
        const rclcpp::Time &time, const rclcpp::Duration &period)
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
        delta_seconds = period.seconds();
        time_seconds = time.seconds();
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.position = hw_vehicle_struct_[0].hw_thrust_structs_[i].current_state_.position + 60 * delta_seconds;
        }

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

        hw_vehicle_struct_[0].current_state_.sim_time = time_seconds;
        hw_vehicle_struct_[0].current_state_.sim_period = delta_seconds;

        publishRealtimePoseTransform(time);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    void SimVehicleSystemMultiInterfaceHardware::publishRealtimePoseTransform(const rclcpp::Time &time)
    {
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            // Original pose in NED
            // RVIZ USES NWU
            tf2::Quaternion q_orig, q_rot, q_new;

            q_orig.setW(hw_vehicle_struct_[0].current_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct_[0].current_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct_[0].current_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct_[0].current_state_.orientation_z);

            // Rotate the previous pose by 0 degrees about X
            q_rot.setRPY(0.0, 0.0, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();

            auto &transform = realtime_transform_publisher_->msg_.transforms.front();
            transform.header.frame_id = cfg_.frame_id;
            transform.child_frame_id = cfg_.child_frame_id;
            transform.header.stamp = time;
            transform.transform.translation.x = hw_vehicle_struct_[0].current_state_.position_x;
            transform.transform.translation.y = hw_vehicle_struct_[0].current_state_.position_y;
            transform.transform.translation.z = -hw_vehicle_struct_[0].current_state_.position_z;

            transform.transform.rotation.x = q_new.x();
            transform.transform.rotation.y = q_new.y();
            transform.transform.rotation.z = q_new.z();
            transform.transform.rotation.w = q_new.w();
            realtime_transform_publisher_->unlockAndPublish();
        }
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimVehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
