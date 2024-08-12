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

// #pragma once

#ifndef ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_

#include "ros2_control_blue_reach_5/joint.hpp"
#include "ros2_control_blue_reach_5/vehicle.hpp"

namespace uvms
{
    class State
    {
    private:
    public:
        // Store the state & commands for the robot joints
        std::vector<Joint> hw_joint_struct_;

        casadi::DM tau_covariance = 100 * casadi::DM::eye(4);
        casadi::DM x_covariance = 100 * casadi::DM::eye(8);

        struct Pose_
        {
            double position_x, position_y, position_z;
            double orientation_w, orientation_x, orientation_y, orientation_z;
        };

        Pose_ default_state_, command_state_, current_state_, async_state_;


        struct Step_condition_ {
            double time;
            double t_step;
            double je_ic_position, jd_ic_position, jc_ic_position, jb_ic_position, ja_ic_position;
            double je_ic_velocity, jd_ic_velocity, jc_ic_velocity, jb_ic_velocity, ja_ic_velocity;
            double je_ic_effort, jd_ic_effort, jc_ic_effort, jb_ic_effort, ja_ic_effort;
        };

        Step_condition_ mhe_data;

        // Store the state & commands for the robot vehicle
        blue::dynamics::Vehicle hw_vehicle_struct_;

        State() = default;
    };
} // namespace uvms
#endif // ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_
