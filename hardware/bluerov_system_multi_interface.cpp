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

#include "ros2_control_blue_reach_5/bluerov_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/dvldriver.hpp"

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
#include <rclcpp/qos.hpp> // Ensure this header is included

using namespace casadi;

namespace
{
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace ros2_control_blue_reach_5
{
    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Access the name from the HardwareInfo
        system_name = info_.name;
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");

        hw_vehicle_struct.frame_id = info_.hardware_parameters["frame_id"];
        hw_vehicle_struct.child_frame_id = info_.hardware_parameters["child_frame_id"];
        hw_vehicle_struct.robot_prefix = info_.hardware_parameters["prefix"];
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************robot prefix: %s", hw_vehicle_struct.robot_prefix.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************frame id: %s", hw_vehicle_struct.frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************child frame id: %s", hw_vehicle_struct.child_frame_id.c_str());

        blue::dynamics::Vehicle::Pose_vel initial_state{
            5.0, 5.0, 2.0,      // Randomized position: x, y, z
            1.0, 0.0, 0.0, 0.0, // Orientation: qw, qx, qy, qz
            0.0, 0.0, 0.0,      // Linear velocities: vx, vy, vz
            0.0, 0.0, 0.0,      // Angular velocities: wx, wy, wz
            0.0, 0.0, 0.0,      // Forces: Fx, Fy, Fz
            0.0, 0.0, 0.0       // Torques: Tx, Ty, Tz
        };

        hw_vehicle_struct.set_vehicle_name("blue ROV heavy 0", initial_state);

        hw_vehicle_struct.thrustSizeAllocation(info_.joints.size());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            Thruster::State defaultState{0.0, 0.0, 0.0, 0.0};
            hw_vehicle_struct.hw_thrust_structs_.emplace_back(joint.name, defaultState);
            // RRBotSystemMultiInterface has exactly 6 joint state interfaces
            if (joint.state_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
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
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 27 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // RRBotSystemMultiInterface has exactly 25 gpio command interfaces
            if (gpio.command_interfaces.size() != 25)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu command interfaces. 25 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // setup realtime buffers, ROS publishers ...
        try
        {
            // Initialize node
            node_topics_interface_ = std::make_shared<rclcpp::Node>(system_name + "_topics_interface");

            // Initialize executor and add node
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_topics_interface_);

            // Start spinning in a separate thread
            spin_thread_ = std::thread([this]()
                                       { executor_->spin(); });

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Started executor and spinning node_topics_interface.");

            // tf publisher
            transform_publisher_ = rclcpp::create_publisher<tf>(node_topics_interface_, DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(
                    transform_publisher_);

            auto &transform_message = realtime_transform_publisher_->msg_;
            transform_message.transforms.resize(2);

            // Setup IMU subscription with Reliable QoS for testing
            auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
            auto callback =
                [this](const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) -> void
            {
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received IMU message");
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    rt_imu_subscriber__ptr_.writeFromNonRT(imu_msg);
                    imu_new_msg_ = true;
                }
            };

            imu_subscriber_ =
                node_topics_interface_->create_subscription<sensor_msgs::msg::Imu>(
                    "/mavros/imu/data", best_effort_qos, callback);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /mavros/imu/data with Reliable QoS");

            // Initialize the realtime dvl publisher
            dvl_velocity_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(node_topics_interface_, "/dvl/velocity", rclcpp::SystemDefaultsQoS());
            realtime_dvl_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(
                    dvl_velocity_publisher_);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "DVL velocity realtime publisher initialized on topic /dvl/velocity.");

            dvl_driver_.subscribe([this](const nlohmann::json &msg)
                                  {
                // This lambda runs in the DVLDriver poll thread whenever new JSON arrives

                // Deserialize into DVLMessage
                dvl_msg = blue::dynamics::DVLMessage::from_json(msg);

                // Lock the mutex to safely update shared data
                std::lock_guard<std::mutex> lock(dvl_data_mutex_);

                switch (dvl_msg.message_type) {
                    case blue::dynamics::DVLMessageType::VELOCITY: {
                        dv_vel = std::get<blue::dynamics::DVLVelocityMessage>(dvl_msg.data);
                        hw_vehicle_struct.dvl_state.altitude = dv_vel.altitude;
                        hw_vehicle_struct.dvl_state.fom = dv_vel.fom;
                        hw_vehicle_struct.dvl_state.format = dv_vel.format;
                        hw_vehicle_struct.dvl_state.status = dv_vel.status;
                        hw_vehicle_struct.dvl_state.time = dv_vel.time;
                        hw_vehicle_struct.dvl_state.covariance = dv_vel.covariance;
                        hw_vehicle_struct.dvl_state.time_of_transmission = dv_vel.time_of_transmission;
                        hw_vehicle_struct.dvl_state.time_of_validity = dv_vel.time_of_validity;
                        hw_vehicle_struct.dvl_state.transducers = dv_vel.transducers;
                        hw_vehicle_struct.dvl_state.type = dv_vel.type;
                        hw_vehicle_struct.dvl_state.velocity_valid = dv_vel.velocity_valid;
                        hw_vehicle_struct.dvl_state.vx = dv_vel.vx;
                        hw_vehicle_struct.dvl_state.vy = dv_vel.vy;
                        hw_vehicle_struct.dvl_state.vz = dv_vel.vz;

                        // Set flag to indicate new data is ready
                        new_dvl_data_available_ = true;
                        break;
                    }
                    case blue::dynamics::DVLMessageType::POSITION_LOCAL: {
                        dv_pose = std::get<blue::dynamics::DVLPoseMessage>(dvl_msg.data);
                        hw_vehicle_struct.dvl_state.format = dv_pose.format;
                        hw_vehicle_struct.dvl_state.pitch = dv_pose.pitch;
                        hw_vehicle_struct.dvl_state.roll = dv_pose.roll;
                        hw_vehicle_struct.dvl_state.status = dv_pose.status;
                        hw_vehicle_struct.dvl_state.std_dev = dv_pose.std_dev;
                        hw_vehicle_struct.dvl_state.ts = dv_pose.ts;
                        hw_vehicle_struct.dvl_state.type = dv_pose.type;
                        hw_vehicle_struct.dvl_state.x = dv_pose.x;
                        hw_vehicle_struct.dvl_state.y = dv_pose.y;
                        hw_vehicle_struct.dvl_state.yaw = dv_pose.yaw;
                        hw_vehicle_struct.dvl_state.z = dv_pose.z;
                        break;
                    }
                    case blue::dynamics::DVLMessageType::UNKNOWN:
                    default:
                        RCLCPP_WARN(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received unknown DVL message type.");
                        break;
                } });

            dvl_driver_.start("192.168.2.95", 16171);
        }
        catch (const std::exception &e)
        {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
                e.what());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "configure successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    BlueRovSystemMultiInterfaceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.acceleration));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.current));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period));
        }

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &hw_vehicle_struct.current_state_.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &hw_vehicle_struct.current_state_.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &hw_vehicle_struct.current_state_.position_z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &hw_vehicle_struct.current_state_.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &hw_vehicle_struct.current_state_.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &hw_vehicle_struct.current_state_.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &hw_vehicle_struct.current_state_.orientation_z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[7].name, &hw_vehicle_struct.current_state_.u));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[8].name, &hw_vehicle_struct.current_state_.v));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[9].name, &hw_vehicle_struct.current_state_.w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[10].name, &hw_vehicle_struct.current_state_.p));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[11].name, &hw_vehicle_struct.current_state_.q));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[12].name, &hw_vehicle_struct.current_state_.r));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[13].name, &hw_vehicle_struct.current_state_.du));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[14].name, &hw_vehicle_struct.current_state_.dv));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[15].name, &hw_vehicle_struct.current_state_.dw));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[16].name, &hw_vehicle_struct.current_state_.dp));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[17].name, &hw_vehicle_struct.current_state_.dq));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[18].name, &hw_vehicle_struct.current_state_.dr));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[19].name, &hw_vehicle_struct.current_state_.Fx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[20].name, &hw_vehicle_struct.current_state_.Fy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[21].name, &hw_vehicle_struct.current_state_.Fz));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[22].name, &hw_vehicle_struct.current_state_.Tx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[23].name, &hw_vehicle_struct.current_state_.Ty));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[24].name, &hw_vehicle_struct.current_state_.Tz));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[25].name, &hw_vehicle_struct.current_state_.sim_time));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[26].name, &hw_vehicle_struct.current_state_.sim_period));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BlueRovSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &hw_vehicle_struct.command_state_.position_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &hw_vehicle_struct.command_state_.position_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &hw_vehicle_struct.command_state_.position_z));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &hw_vehicle_struct.command_state_.orientation_w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &hw_vehicle_struct.command_state_.orientation_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &hw_vehicle_struct.command_state_.orientation_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &hw_vehicle_struct.command_state_.orientation_z));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[7].name, &hw_vehicle_struct.command_state_.u));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[8].name, &hw_vehicle_struct.command_state_.v));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[9].name, &hw_vehicle_struct.command_state_.w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[10].name, &hw_vehicle_struct.command_state_.p));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[11].name, &hw_vehicle_struct.command_state_.q));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[12].name, &hw_vehicle_struct.command_state_.r));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[13].name, &hw_vehicle_struct.command_state_.du));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[14].name, &hw_vehicle_struct.command_state_.dv));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[15].name, &hw_vehicle_struct.command_state_.dw));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[16].name, &hw_vehicle_struct.command_state_.dp));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[17].name, &hw_vehicle_struct.command_state_.dq));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[18].name, &hw_vehicle_struct.command_state_.dr));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[19].name, &hw_vehicle_struct.command_state_.Fx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[20].name, &hw_vehicle_struct.command_state_.Fy));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[21].name, &hw_vehicle_struct.command_state_.Fz));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[22].name, &hw_vehicle_struct.command_state_.Tx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[23].name, &hw_vehicle_struct.command_state_.Ty));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[24].name, &hw_vehicle_struct.command_state_.Tz));

        return command_interfaces;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::prepare_command_mode_switch(
        const std::vector<std::string> & /*start_interfaces*/,
        const std::vector<std::string> & /*stop_interfaces*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Command Mode Switch successful");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Activating... please wait...");

        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position = 0.0;
            }
            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].current_state_.velocity))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].current_state_.velocity = 0.0;
            }
            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].current_state_.current))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].current_state_.current = 0.0;
            }
            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].current_state_.acceleration))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].current_state_.acceleration = 0.0;
            }

            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].command_state_.current))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].command_state_.current = 0.0;
            }
            if (std::isnan(hw_vehicle_struct.hw_thrust_structs_[i].command_state_.effort))
            {
                hw_vehicle_struct.hw_thrust_structs_[i].command_state_.effort = 0.0;
            }
        }

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "System successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Deactivating... please wait...");
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        // Lock and check if new data is available
        std::lock_guard<std::mutex> lock(dvl_data_mutex_);
        if (new_dvl_data_available_)
        {
            publishDVLVelocity();
            new_dvl_data_available_ = false;
        }

        sensor_msgs::msg::Imu imu_for_transform;
        sensor_msgs::msg::Imu last_imu_msg_;
        bool have_new_imu = false;

        if (imu_new_msg_)
        {
            auto latest_imu_ptr = rt_imu_subscriber__ptr_.readFromRT();
            if (latest_imu_ptr)
            {
                // Copy the IMU from pointer into our local variable
                // imu_for_transform = *latest_imu_ptr;
                imu_for_transform = **latest_imu_ptr; // Double dereference
                have_new_imu = true;

                // Update the last received IMU message
                last_imu_msg_ = imu_for_transform;
            }
            else
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "No valid IMU data pointer available from rt_imu_subscriber__ptr_");
            }
            imu_new_msg_ = false;
        }
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position = hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position + 60 * delta_seconds;
        }

        hw_vehicle_struct.current_state_.position_x = hw_vehicle_struct.command_state_.position_x;
        hw_vehicle_struct.current_state_.position_y = hw_vehicle_struct.command_state_.position_y;
        hw_vehicle_struct.current_state_.position_z = hw_vehicle_struct.command_state_.position_z;
        hw_vehicle_struct.current_state_.orientation_w = hw_vehicle_struct.command_state_.orientation_w;
        hw_vehicle_struct.current_state_.orientation_x = hw_vehicle_struct.command_state_.orientation_x;
        hw_vehicle_struct.current_state_.orientation_y = hw_vehicle_struct.command_state_.orientation_y;
        hw_vehicle_struct.current_state_.orientation_z = hw_vehicle_struct.command_state_.orientation_z;

        hw_vehicle_struct.current_state_.u = hw_vehicle_struct.command_state_.u;
        hw_vehicle_struct.current_state_.v = hw_vehicle_struct.command_state_.v;
        hw_vehicle_struct.current_state_.w = hw_vehicle_struct.command_state_.w;
        hw_vehicle_struct.current_state_.p = hw_vehicle_struct.command_state_.p;
        hw_vehicle_struct.current_state_.q = hw_vehicle_struct.command_state_.q;
        hw_vehicle_struct.current_state_.r = hw_vehicle_struct.command_state_.r;

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = -hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.current_state_.sim_time = time_seconds;
        hw_vehicle_struct.current_state_.sim_period = delta_seconds;
        // Publish transforms
        if (have_new_imu)
        {
            publishRealtimePoseTransform(time, imu_for_transform);
        }
        else
        {
            // Optionally, publish pose transform without IMU data
            publishRealtimePoseTransform(time, last_imu_msg_);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }

    void BlueRovSystemMultiInterfaceHardware::publishRealtimePoseTransform(const rclcpp::Time &time,
                                                                           const sensor_msgs::msg::Imu &imu_msg)
    {
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            auto &transforms = realtime_transform_publisher_->msg_.transforms;
            // Original pose in NED
            // RVIZ USES NWU
            tf2::Quaternion q_orig, q_rot, q_new;

            q_orig.setW(hw_vehicle_struct.current_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct.current_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct.current_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct.current_state_.orientation_z);

            // Rotate the previous pose by 0 degrees about X
            q_rot.setRPY(0.0, 0.0, 0.0);
            q_new = q_rot * q_orig;
            q_new.normalize();

            auto &transform = transforms[0];
            transform.header.frame_id = hw_vehicle_struct.frame_id;
            transform.child_frame_id = hw_vehicle_struct.child_frame_id;
            transform.header.stamp = time;
            transform.transform.translation.x = hw_vehicle_struct.current_state_.position_x;
            transform.transform.translation.y = hw_vehicle_struct.current_state_.position_y;
            transform.transform.translation.z = -hw_vehicle_struct.current_state_.position_z;

            transform.transform.rotation.x = q_new.x();
            transform.transform.rotation.y = q_new.y();
            transform.transform.rotation.z = q_new.z();
            transform.transform.rotation.w = q_new.w();

            // We assume transform_message.transforms.resize(2) in on_configure()
            // The second transform index is [1]
            auto &imu_tf = transforms[1];
            // Set timestamps
            imu_tf.header.stamp = time;
            imu_tf.header.frame_id = hw_vehicle_struct.child_frame_id; // <- parent frame, adjust as desired
            imu_tf.child_frame_id = "imu_link";              // <- the IMU frame you want to broadcast

            // Fill orientation from the subscribed IMU orientation
            imu_tf.transform.translation.x = 0.0;
            imu_tf.transform.translation.y = 0.0;
            imu_tf.transform.translation.z = 0.0;

            imu_tf.transform.rotation.x = imu_msg.orientation.x;
            imu_tf.transform.rotation.y = imu_msg.orientation.y;
            imu_tf.transform.rotation.z = imu_msg.orientation.z;
            imu_tf.transform.rotation.w = imu_msg.orientation.w;

            // Publish the TF
            realtime_transform_publisher_->unlockAndPublish();
        }
    }
    void BlueRovSystemMultiInterfaceHardware::publishDVLVelocity()
    {
        // Attempt to acquire the lock for real-time publishing
        if (realtime_dvl_velocity_publisher_ && realtime_dvl_velocity_publisher_->trylock())
        {
            // Safely access the message within the realtime publisher
            auto &twist_msg = realtime_dvl_velocity_publisher_->msg_;
            twist_msg.header.stamp = rclcpp::Clock().now();
            twist_msg.header.frame_id = hw_vehicle_struct.frame_id;

            // Assign DVL velocity data
            twist_msg.twist.twist.linear.x = hw_vehicle_struct.dvl_state.vx;
            twist_msg.twist.twist.linear.y = hw_vehicle_struct.dvl_state.vy;
            twist_msg.twist.twist.linear.z = hw_vehicle_struct.dvl_state.vz;

            // Convert 3x3 covariance to 6x6 and assign
            twist_msg.twist.covariance = convert3x3To6x6Covariance(hw_vehicle_struct.dvl_state.covariance);

            // Publish the message safely
            realtime_dvl_velocity_publisher_->unlockAndPublish();
            // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Published DVL velocity with covariance.");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Failed to acquire lock for DVL velocity publishing.");
        }
    }

    // Convert 3x3 covariance to 6x6 format
    std::array<double, 36> BlueRovSystemMultiInterfaceHardware::convert3x3To6x6Covariance(const blue::dynamics::Covariance &linear_cov)
    {
        std::array<double, 36> full_covariance = {0.0};
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                full_covariance[i * 6 + j] = linear_cov.data[i * 3 + j];
            }
        }
        return full_covariance;
    }

    ros2_control_blue_reach_5::BlueRovSystemMultiInterfaceHardware::~BlueRovSystemMultiInterfaceHardware()
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Executor stopped and spin thread joined.");
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Cleaned up executor and spin thread.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::BlueRovSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
