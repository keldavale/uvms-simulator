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

#ifndef ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/subscription.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"

#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/utils.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "ros2_control_blue_reach_5/dvldriver.hpp"

#include <sensor_msgs/msg/imu.hpp> // Add this line
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <mutex>
#include "tf2_ros/static_transform_broadcaster.h"
#include <casadi/casadi.hpp>

namespace ros2_control_blue_reach_5
{

    class BlueRovSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        ~BlueRovSystemMultiInterfaceHardware();
        RCLCPP_SHARED_PTR_DEFINITIONS(BlueRovSystemMultiInterfaceHardware);

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
        // Store the utils function for the robot joints
        casadi_reach_alpha_5::Utils utils_service;

        // Store the state & commands for the robot vehicle
        blue::dynamics::Vehicle hw_vehicle_struct;
        std::string system_name;

        double odom_position_x, odom_position_y, odom_position_z;
        double odom_orientaion_w, odom_orientaion_x, odom_orientaion_y, odom_orientaion_z;

        tf2::Quaternion q_orig, q_rot, q_new;

        using tf = tf2_msgs::msg::TFMessage;

        std::shared_ptr<rclcpp::Publisher<tf>> transform_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf>>
            realtime_transform_publisher_;

        void publishRealtimePoseTransform(
        const rclcpp::Time &time
        );

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;


        void publishDVLVelocity(const rclcpp::Time &time);
        std::array<double, 36> convert3x3To6x6Covariance(const blue::dynamics::Covariance &linear_cov);

        double delta_seconds;
        double time_seconds;

        // DVL driver instance
        a50dvl::driver::DVLDriver dvl_driver_;

        // Mutex to protect DVL data
        std::mutex dvl_data_mutex_;

        blue::dynamics::DVLMessage dvl_msg;
        blue::dynamics::DVLVelocityMessage dv_vel;
        blue::dynamics::DVLPoseMessage dvl_pose;

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> dvl_velocity_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>
            realtime_dvl_velocity_publisher_;

        // Add a flag for data readiness (in the header file)
        bool new_dvl_data_available_ = false;

        // Subscriber for MAVROS IMU data
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        bool imu_new_msg_ = false;

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread spin_thread_;
        std::shared_ptr<rclcpp::Node> node_topics_interface_;

        // Mutex for thread-safe IMU data access
        std::mutex imu_mutex_;
    };

} // namespace ros2_control_blue
#endif // ROS2_CONTROL_BLUE_REACH_5__BLUEROV_SYSTEM_MULTI_INTERFACE_HPP_
