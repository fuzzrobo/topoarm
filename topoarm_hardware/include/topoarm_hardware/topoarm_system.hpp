// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim
// Modified by: Nagashima

#ifndef topoarm_HARDWARE__topoarm_SYSTEM_HPP_
#define topoarm_HARDWARE__topoarm_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "topoarm_msgs/msg/dynamixel_command_effort.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "topoarm_hardware/dynamixel_handler.hpp"
#include "topoarm_hardware/visibility_control.h"

namespace fuzzrobo {
namespace topoarm_hardware {
struct MimicInfo {
    string joint_name;
    double multiplier;
    double ref_pos;
    double ref_vel;
    double ref_curr;
    double state_pos;
    double state_vel;
    double state_curr;
    Dynamixel* original_dxl_ptr;
};

class ToPoArmSystemHardware : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ToPoArmSystemHardware);

    TOPOARM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    TOPOARM_HARDWARE_PUBLIC
    vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TOPOARM_HARDWARE_PUBLIC
    vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TOPOARM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    TOPOARM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    TOPOARM_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    TOPOARM_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    string usb_port_;
    uint32_t baud_rate_;
    uint32_t latency_timer_;
    uint32_t min_id_;
    uint32_t max_id_;
    bool torque_;
    bool torque_retention_;
    double default_profile_acceleration_;
    double default_profile_velocity_;
    vector<DynamixelEEPROM> dxl_eeproms_;

    unique_ptr<DynamixelHandler> handler_;
    vector<MimicInfo> mimic_infos_;

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    vector<rclcpp::Subscription<topoarm_msgs::msg::DynamixelCommandEffort>::SharedPtr> effort_subscribers_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger logger_ = rclcpp::get_logger("topoarm_hardware");

    void effort_command_callback(const topoarm_msgs::msg::DynamixelCommandEffort::SharedPtr msg);
    void check_hardware_error();
};
}  // namespace topoarm_hardware
}  // namespace fuzzrobo
#endif  // topoarm_HARDWARE__topoarm_SYSTEM_HPP_
