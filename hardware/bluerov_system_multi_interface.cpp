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

        if (info_.hardware_parameters.find("frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.frame_id = info_.hardware_parameters["frame_id"];

        if (info_.hardware_parameters.find("child_frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'child_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.child_frame_id = info_.hardware_parameters["child_frame_id"];

        if (info_.hardware_parameters.find("map_frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'map_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.map_frame_id = info_.hardware_parameters["map_frame_id"];

        if (info_.hardware_parameters.find("prefix") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'prefix' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.robot_prefix = info_.hardware_parameters["prefix"];

        // Get the maximum number of attempts that can be made to set the thruster parameters before failing
        if (info_.hardware_parameters.find("max_set_param_attempts") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'max_set_param_attempts' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************robot prefix: %s", hw_vehicle_struct.robot_prefix.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************frame id: %s", hw_vehicle_struct.frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************child frame id: %s", hw_vehicle_struct.child_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************map frame id: %s", hw_vehicle_struct.map_frame_id.c_str());

        map_position_x = 5.0;
        map_position_y = 5.0;
        map_position_z = 0.0;

        map_orientaion_w = 1.0;
        map_orientaion_x = 0.0;
        map_orientaion_y = 0.0;
        map_orientaion_z = 0.0;

        blue::dynamics::Vehicle::Pose_vel initial_state{
            0.0, 0.0, 0.0,      // map_frame at position: x, y, z
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
            // Make sure the the joint-level parameters exist
            if (
                joint.parameters.find("param_name") == joint.parameters.cend() ||
                joint.parameters.find("default_param_value") == joint.parameters.cend() ||
                joint.parameters.find("channel") == joint.parameters.cend() ||
                joint.parameters.find("neutral_pwm") == joint.parameters.cend())
            {
                RCLCPP_ERROR( // NOLINT
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Joint %s missing required configurations. Ensure that the `param_name`, `default_param_value`, `neutral_pwm` and "
                    "`channel` are provided for each joint.",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            };

            rcl_interfaces::msg::Parameter mavros_rc_param;
            mavros_rc_param.name = joint.parameters.at("param_name");
            mavros_rc_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            mavros_rc_param.value.integer_value = std::stoi(joint.parameters.at("default_param_value"));

            int rc_channel = std::stoi(joint.parameters.at("channel"));
            int rc_neutral_pwm = std::stoi(joint.parameters.at("neutral_pwm"));

            Thruster::State defaultState{};
            hw_vehicle_struct.hw_thrust_structs_.emplace_back(joint.name, mavros_rc_param, rc_channel, rc_neutral_pwm, defaultState);
            // RRBotSystemMultiInterface has exactly 6 joint state interfaces
            if (joint.state_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu state interfaces. 6 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };

            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu command interfaces. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };
        };
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************hw_vehicle_struct.hw_thrust_structs_.size()s: %zu",
                    hw_vehicle_struct.hw_thrust_structs_.size());

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // RRBotSystemMultiInterface has exactly 37 gpio state interfaces
            if (gpio.state_interfaces.size() != 37)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 37 expected.", gpio.name.c_str(),
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
                        "Started executor and spinning node_topics_interface: %s", node_topics_interface_->get_name());

            // Initialize the StaticTransformBroadcaster
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_topics_interface_);

            // override_rc publisher
            override_rc_pub_ = rclcpp::create_publisher<mavros_msgs::msg::OverrideRCIn>(node_topics_interface_,
                                                                                        "mavros/rc/override",
                                                                                        rclcpp::SystemDefaultsQoS());

            rt_override_rc_pub_ = std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

            rt_override_rc_pub_->lock();
            for (auto &channel : rt_override_rc_pub_->msg_.channels)
            {
                channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
            }
            rt_override_rc_pub_->unlock();

            // Assuming node_topics_interface_ is a shared_ptr<rclcpp::Node>
            set_params_client_ = node_topics_interface_->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");

            using namespace std::chrono_literals;
            while (!set_params_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR( // NOLINT
                        rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Interrupted while waiting for the `mavros/set_parameters` service.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                RCLCPP_INFO( // NOLINT
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Waiting for the `mavros/set_parameters` service to be available...");
            }

            // tf publisher
            transform_publisher_ = rclcpp::create_publisher<tf>(node_topics_interface_,
                                                                DEFAULT_TRANSFORM_TOPIC,
                                                                rclcpp::SystemDefaultsQoS());
            realtime_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(
                    transform_publisher_);

            auto &transform_message = realtime_transform_publisher_->msg_;
            transform_message.transforms.resize(1);

            // Setup IMU subscription with Reliable QoS for testing
            auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
            auto imu_callback =
                [this](const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) -> void
            {
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received IMU message");
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    hw_vehicle_struct.imu_state.position_x = 0.0;
                    hw_vehicle_struct.imu_state.position_y = 0.0;
                    hw_vehicle_struct.imu_state.position_z = 0.0;

                    hw_vehicle_struct.imu_state.orientation_w = imu_msg->orientation.w;
                    hw_vehicle_struct.imu_state.orientation_x = imu_msg->orientation.x;
                    hw_vehicle_struct.imu_state.orientation_y = imu_msg->orientation.y;
                    hw_vehicle_struct.imu_state.orientation_z = imu_msg->orientation.z;
                    imu_new_msg_ = true;
                }
            };

            imu_subscriber_ =
                node_topics_interface_->create_subscription<sensor_msgs::msg::Imu>(
                    "/mavros/imu/data", best_effort_qos, imu_callback);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /mavros/imu/data with Reliable QoS");

            // Setup Kalman filtered odom subscription
            auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

            // Define the odometry callback
            auto filtered_odom_callback =
                [this](const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg) -> void
            {
                // Log receipt of odometry message
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received filtered odom message");

                {
                    std::lock_guard<std::mutex> lock_odom(filtered_odom_mutex_);
                    hw_vehicle_struct.async_state_.position_x = odom_msg->pose.pose.position.x;
                    hw_vehicle_struct.async_state_.position_y = odom_msg->pose.pose.position.y;
                    hw_vehicle_struct.async_state_.position_z = odom_msg->pose.pose.position.z;
                    hw_vehicle_struct.async_state_.orientation_w = odom_msg->pose.pose.orientation.w;
                    hw_vehicle_struct.async_state_.orientation_x = odom_msg->pose.pose.orientation.x;
                    hw_vehicle_struct.async_state_.orientation_y = odom_msg->pose.pose.orientation.y;
                    hw_vehicle_struct.async_state_.orientation_z = odom_msg->pose.pose.orientation.z;

                    hw_vehicle_struct.async_state_.u = odom_msg->twist.twist.linear.x;
                    hw_vehicle_struct.async_state_.v = odom_msg->twist.twist.linear.y;
                    hw_vehicle_struct.async_state_.w = odom_msg->twist.twist.linear.z;
                    hw_vehicle_struct.async_state_.p = odom_msg->twist.twist.angular.x;
                    hw_vehicle_struct.async_state_.q = odom_msg->twist.twist.angular.y;
                    hw_vehicle_struct.async_state_.r = odom_msg->twist.twist.angular.z;
                    filtered_odom_new_msg_ = true;
                }
            };

            // Create the Odometry subscriber
            filterd_odom_subscriber_ =
                node_topics_interface_->create_subscription<nav_msgs::msg::Odometry>(
                    "/blue_rov/odom", // Updated topic name for odometry
                    reliable_qos,     // Use reliable QoS as per the log message
                    filtered_odom_callback);

            // Log the subscription
            RCLCPP_INFO(
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                "Subscribed to /blue_rov/odom_ned with Reliable QoS");

            // Initialize the realtime dvl publisher
            dvl_velocity_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(node_topics_interface_,
                                                                                                               "/dvl/twist", rclcpp::SystemDefaultsQoS());
            realtime_dvl_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(
                    dvl_velocity_publisher_);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "DVL velocity realtime publisher initialized on topic /dvl/twist.");

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
                        dvl_pose = std::get<blue::dynamics::DVLPoseMessage>(dvl_msg.data);
                        hw_vehicle_struct.dvl_state.format = dvl_pose.format;
                        hw_vehicle_struct.dvl_state.pitch = dvl_pose.pitch;
                        hw_vehicle_struct.dvl_state.roll = dvl_pose.roll;
                        hw_vehicle_struct.dvl_state.status = dvl_pose.status;
                        hw_vehicle_struct.dvl_state.std_dev = dvl_pose.std_dev;
                        hw_vehicle_struct.dvl_state.ts = dvl_pose.ts;
                        hw_vehicle_struct.dvl_state.type = dvl_pose.type;
                        hw_vehicle_struct.dvl_state.x = dvl_pose.x;
                        hw_vehicle_struct.dvl_state.y = dvl_pose.y;
                        hw_vehicle_struct.dvl_state.yaw = dvl_pose.yaw;
                        hw_vehicle_struct.dvl_state.z = dvl_pose.z;
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
                info_.joints[i].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.rc_pwm));
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
            info_.gpios[0].name, info_.gpios[0].state_interfaces[25].name, &hw_vehicle_struct.sim_time));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[26].name, &hw_vehicle_struct.sim_period));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[27].name, &hw_vehicle_struct.imu_state.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[28].name, &hw_vehicle_struct.imu_state.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[29].name, &hw_vehicle_struct.imu_state.position_z));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[30].name, &hw_vehicle_struct.imu_state.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[31].name, &hw_vehicle_struct.imu_state.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[32].name, &hw_vehicle_struct.imu_state.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[33].name, &hw_vehicle_struct.imu_state.orientation_z));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[34].name, &hw_vehicle_struct.dvl_state.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[35].name, &hw_vehicle_struct.dvl_state.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[36].name, &hw_vehicle_struct.dvl_state.yaw));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BlueRovSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[1].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[2].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[3].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[4].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[5].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[6].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[7].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm));

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

        publishStaticPoseTransform();
        // Prepare parameters to set thruster to RC passthrough
        std::vector<rcl_interfaces::msg::Parameter> params;
        params.reserve(hw_vehicle_struct.hw_thrust_structs_.size());

        for (const auto &config : hw_vehicle_struct.hw_thrust_structs_)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Setting parameter: %s", config.param.name.c_str());
            rcl_interfaces::msg::Parameter param;
            param.name = config.param.name;
            param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            param.value.integer_value = 1; // Set to RC passthrough
            params.emplace_back(param);
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters = params;

        // Send the service request with a callback
        auto future = set_params_client_->async_send_request(
            request,
            [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
            {
                std::lock_guard<std::mutex> lock(activate_mutex_);
                const auto responses = future.get()->results;
                for (const auto &response : responses)
                {
                    if (!response.successful)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Failed to set thruster parameter: %s", response.reason.c_str());
                        activation_successful_ = false;
                        activation_complete_ = true;
                        activate_cv_.notify_one();
                        return;
                    }
                }

                RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Successfully set thruster parameters to RC passthrough!");

                // Stop the thrusters before switching to an external controller
                stop_thrusters();

                activation_successful_ = true;
                activation_complete_ = true;
                activate_cv_.notify_one();
            });

        // Wait for the activation to complete
        {
            std::unique_lock<std::mutex> lock(activate_mutex_);
            activate_cv_.wait(lock, [this]()
                              { return activation_complete_; });
        }

        if (!activation_successful_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                         "Failed to set thruster parameters to passthrough mode.");
            return hardware_interface::CallbackReturn::ERROR;
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

        // Stop the thrusters before switching out of passthrough mode
        stop_thrusters();

        // Prepare parameters to set thruster to default
        std::vector<rcl_interfaces::msg::Parameter> default_params;
        default_params.reserve(hw_vehicle_struct.hw_thrust_structs_.size());

        for (const auto &config : hw_vehicle_struct.hw_thrust_structs_)
        {
            default_params.emplace_back(config.param);
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters = default_params;

        // Send the service request with a callback
        auto future = set_params_client_->async_send_request(
            request,
            [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
            {
                std::lock_guard<std::mutex> lock(deactivate_mutex_);
                const auto responses = future.get()->results;
                for (const auto &response : responses)
                {
                    if (!response.successful)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Failed to set default thruster parameter: %s", response.reason.c_str());
                        deactivation_successful_ = false;
                        deactivation_complete_ = true;
                        deactivate_cv_.notify_one();
                        return;
                    }
                }

                RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Successfully restored the default thruster parameter!");

                deactivation_successful_ = true;
                deactivation_complete_ = true;
                deactivate_cv_.notify_one();
            });

        // Wait for the activation to complete
        {
            std::unique_lock<std::mutex> lock(deactivate_mutex_);
            deactivate_cv_.wait(lock, [this]()
                                { return deactivation_complete_; });
        }

        if (!deactivation_successful_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                         "Failed to set default thruster parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }
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
            publishDVLVelocity(time);
            new_dvl_data_available_ = false;
        }
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position += 60 * delta_seconds;
        }

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = -hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = -hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.sim_time = time_seconds;
        hw_vehicle_struct.sim_period = delta_seconds;

        // Lock and check if new data is available
        std::lock_guard<std::mutex> lock_odom(filtered_odom_mutex_);
        if (filtered_odom_new_msg_)
        {
            hw_vehicle_struct.current_state_.position_x = hw_vehicle_struct.async_state_.position_x;
            hw_vehicle_struct.current_state_.position_y = hw_vehicle_struct.async_state_.position_y;
            hw_vehicle_struct.current_state_.position_z = hw_vehicle_struct.async_state_.position_z;
            hw_vehicle_struct.current_state_.orientation_w = hw_vehicle_struct.async_state_.orientation_w;
            hw_vehicle_struct.current_state_.orientation_x = hw_vehicle_struct.async_state_.orientation_x;
            hw_vehicle_struct.current_state_.orientation_y = hw_vehicle_struct.async_state_.orientation_y;
            hw_vehicle_struct.current_state_.orientation_z = hw_vehicle_struct.async_state_.orientation_z;

            hw_vehicle_struct.current_state_.u = hw_vehicle_struct.async_state_.u;
            hw_vehicle_struct.current_state_.v = hw_vehicle_struct.async_state_.v;
            hw_vehicle_struct.current_state_.w = hw_vehicle_struct.async_state_.w;
            hw_vehicle_struct.current_state_.p = hw_vehicle_struct.async_state_.p;
            hw_vehicle_struct.current_state_.q = hw_vehicle_struct.async_state_.q;
            hw_vehicle_struct.current_state_.r = hw_vehicle_struct.async_state_.r;
            filtered_odom_new_msg_ = false;
        }

        // Publish transforms
        publishRealtimePoseTransform(time);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        // {
        //     for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
        //     {
        //         rt_override_rc_pub_->msg_.channels[thruster_configs_[i].channel - 1] = static_cast<int>(hw_vehicle_struct.hw_thrust_structs_[i].command_state_.command_pwm);
        //     }
        //     rt_override_rc_pub_->unlockAndPublish();
        // }
        return hardware_interface::return_type::OK;
    }

    void BlueRovSystemMultiInterfaceHardware::publishStaticPoseTransform()
    {
        // Capture the current time
        rclcpp::Time current_time = node_topics_interface_->now();

        // Create and send the static map transform
        geometry_msgs::msg::TransformStamped static_map_transform;

        static_map_transform.header.stamp = current_time;
        static_map_transform.header.frame_id = hw_vehicle_struct.frame_id;
        static_map_transform.child_frame_id = hw_vehicle_struct.map_frame_id;

        // Set translation based on current state
        static_map_transform.transform.translation.x = map_position_x;
        static_map_transform.transform.translation.y = map_position_y;
        static_map_transform.transform.translation.z = map_position_z;

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "current attitude from KF odom : %f %f %f %f",
            hw_vehicle_struct.async_state_.orientation_w,
            hw_vehicle_struct.async_state_.orientation_x,
            hw_vehicle_struct.async_state_.orientation_y,
            hw_vehicle_struct.async_state_.orientation_z);

        // Set rotation based on current state (quaternion)
        static_map_transform.transform.rotation.x = map_orientaion_x;
        static_map_transform.transform.rotation.y = map_orientaion_y;
        static_map_transform.transform.rotation.z = map_orientaion_z;
        static_map_transform.transform.rotation.w = map_orientaion_w;
        // Publish the static transform
        static_tf_broadcaster_->sendTransform(static_map_transform);

        // Create and send the static dvl transform
        geometry_msgs::msg::TransformStamped static_dvl_transform;

        static_dvl_transform.header.stamp = current_time;
        static_dvl_transform.header.frame_id = hw_vehicle_struct.child_frame_id;
        static_dvl_transform.child_frame_id = hw_vehicle_struct.robot_prefix + "dvl_link";

        // Set translation based on current state
        static_dvl_transform.transform.translation.x = -0.060;
        static_dvl_transform.transform.translation.y = 0.000;
        static_dvl_transform.transform.translation.z = -0.105;

        // Rotate the pose about X UPRIGHT
        q_rot_dvl.setRPY(0.0, 0.0, 0.0);

        q_rot_dvl.normalize();

        static_dvl_transform.transform.rotation.x = q_rot_dvl.x();
        static_dvl_transform.transform.rotation.y = q_rot_dvl.y();
        static_dvl_transform.transform.rotation.z = q_rot_dvl.z();
        static_dvl_transform.transform.rotation.w = q_rot_dvl.w();

        // Publish the static transform
        static_tf_broadcaster_->sendTransform(static_dvl_transform);

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
            "Published static odom transform once during activation.");
    };

    void BlueRovSystemMultiInterfaceHardware::publishRealtimePoseTransform(const rclcpp::Time &time)
    {
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            auto &transforms = realtime_transform_publisher_->msg_.transforms;
            auto &StateEstimateTransform = transforms.front();
            StateEstimateTransform.header.frame_id = hw_vehicle_struct.map_frame_id;
            StateEstimateTransform.child_frame_id = hw_vehicle_struct.child_frame_id;
            StateEstimateTransform.header.stamp = time;
            StateEstimateTransform.transform.translation.x = -hw_vehicle_struct.current_state_.position_x;
            StateEstimateTransform.transform.translation.y = hw_vehicle_struct.current_state_.position_y;
            StateEstimateTransform.transform.translation.z = hw_vehicle_struct.current_state_.position_z;

            q_orig.setW(hw_vehicle_struct.current_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct.current_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct.current_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct.current_state_.orientation_z);

            q_orig.normalize();

            StateEstimateTransform.transform.rotation.x = q_orig.x();
            StateEstimateTransform.transform.rotation.y = q_orig.y();
            StateEstimateTransform.transform.rotation.z = q_orig.z();
            StateEstimateTransform.transform.rotation.w = q_orig.w();

            // Publish the TF
            realtime_transform_publisher_->unlockAndPublish();
        }
    }
    void BlueRovSystemMultiInterfaceHardware::publishDVLVelocity(const rclcpp::Time &time)
    {
        // Attempt to acquire the lock for real-time publishing
        if (realtime_dvl_velocity_publisher_ && realtime_dvl_velocity_publisher_->trylock())
        {
            // Safely access the message within the realtime publisher
            auto &twist_msg = realtime_dvl_velocity_publisher_->msg_;
            twist_msg.header.stamp = time;
            twist_msg.header.frame_id = hw_vehicle_struct.robot_prefix + "dvl_link";

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

    void BlueRovSystemMultiInterfaceHardware::stop_thrusters()
    {
        if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        {
            for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
            {
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.hw_thrust_structs_[i].channel - 1] = hw_vehicle_struct.hw_thrust_structs_[i].neutral_pwm;
            }
            rt_override_rc_pub_->unlockAndPublish();
        }
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
