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

#include "topoarm_hardware/topoarm_system.hpp"

namespace fuzzrobo {
namespace topoarm_hardware {
bool stringToBool(const std::string& str) {
    std::string lower_str = str;
    // 文字列をすべて小文字に変換
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    
    bool value;
    std::istringstream(lower_str) >> std::boolalpha >> value;
    return value;
}
hardware_interface::CallbackReturn ToPoArmSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Dynamixel Handler
    usb_port_ = info_.hardware_parameters["usb_port"];
    baud_rate_ = stoi(info_.hardware_parameters["baud_rate"]);
    latency_timer_ = stoi(info_.hardware_parameters["latency_timer"]);
    min_id_ = stoi(info_.hardware_parameters["min_id"]);
    max_id_ = stoi(info_.hardware_parameters["max_id"]);
    torque_ = stringToBool(info_.hardware_parameters["torque"]);
    torque_retention_ = stringToBool(info_.hardware_parameters["torque_retention"]);
    default_profile_acceleration_ = stod(info_.hardware_parameters["default_profile_acceleration"]);
    default_profile_velocity_ = stod(info_.hardware_parameters["default_profile_velocity"]);

    // EEPROM デフォルト値
    DynamixelEEPROM default_eeprom;
    default_eeprom.return_delay_time = stoi(info_.hardware_parameters["eeprom.return_delay_time"]);
    default_eeprom.drive_mode = stoi(info_.hardware_parameters["eeprom.drive_mode"]);
    default_eeprom.operating_mode = stoi(info_.hardware_parameters["eeprom.operating_mode"]);
    default_eeprom.homing_offset = stoi(info_.hardware_parameters["eeprom.homing_offset"]);
    default_eeprom.moving_threshold = stoi(info_.hardware_parameters["eeprom.moving_threshold"]);
    default_eeprom.temperature_limit = stoi(info_.hardware_parameters["eeprom.temperature_limit"]);
    default_eeprom.max_voltage_limit = stoi(info_.hardware_parameters["eeprom.max_voltage_limit"]);
    default_eeprom.min_voltage_limit = stoi(info_.hardware_parameters["eeprom.min_voltage_limit"]);
    default_eeprom.pwm_limit = stoi(info_.hardware_parameters["eeprom.pwm_limit"]);
    default_eeprom.current_limit = stoi(info_.hardware_parameters["eeprom.current_limit"]);
    default_eeprom.velocity_limit = stoi(info_.hardware_parameters["eeprom.velocity_limit"]);
    default_eeprom.max_position_limit = stoi(info_.hardware_parameters["eeprom.max_position_limit"]);
    default_eeprom.min_position_limit = stoi(info_.hardware_parameters["eeprom.min_position_limit"]);
    default_eeprom.shutdown = stoi(info_.hardware_parameters["eeprom.shutdown"]);
    for(int i=0;i<info_.joints.size();++i)
        dxl_eeproms_.emplace_back(default_eeprom);

    // パラメータ表示
    RCLCPP_INFO(logger_, "usb_port: %s", usb_port_.c_str());
    RCLCPP_INFO(logger_, "baud_rate: %d", baud_rate_);
    RCLCPP_INFO(logger_, "latency_timer: %d", latency_timer_);
    RCLCPP_INFO(logger_, "min_id: %d", min_id_);
    RCLCPP_INFO(logger_, "max_id: %d", max_id_);
    RCLCPP_INFO(logger_, "torque: %s", torque_ ? "true" : "false");
    RCLCPP_INFO(logger_, "torque_retention: %s", torque_retention_ ? "true" : "false");
    RCLCPP_INFO(logger_, "default_profile_acceleration: %f", default_profile_acceleration_);
    RCLCPP_INFO(logger_, "default_profile_velocity: %f", default_profile_velocity_);

    handler_ = std::make_unique<DynamixelHandler>(usb_port_, baud_rate_, latency_timer_);

    if (handler_->open_port()) {
        RCLCPP_INFO(logger_, "Succeeded to open port %s", usb_port_.c_str());
    } else {
        RCLCPP_FATAL(logger_, "Failed to open port %s", usb_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    vector<string> joint_names;
    vector<double> joint_multipliers, joint_position_mins, joint_position_maxs;
    vector<int> joint_position_kp, joint_position_ki, joint_position_kd;
    for (int i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        if (joint.parameters.find("mimic") != joint.parameters.end()) {
            continue;
        }
        joint_names.push_back(joint.name);
        double multiplier = joint.parameters.find("multiplier") != joint.parameters.end() ? stod(joint.parameters.at("multiplier")) : 1.0;
        joint_multipliers.push_back(multiplier);
        for(auto &command_interface: joint.command_interfaces){
            if(command_interface.name == hardware_interface::HW_IF_POSITION){
                joint_position_mins.push_back(stod(command_interface.min));
                joint_position_maxs.push_back(stod(command_interface.max));
                break;
            }
        }
        for(auto &param: joint.parameters){
            if(param.first == "position_kp"){
                joint_position_kp.emplace_back(stoi(param.second));
            }else if(param.first == "position_ki"){
                joint_position_ki.emplace_back(stoi(param.second));
            }else if(param.first == "position_kd"){
                joint_position_kd.emplace_back(stoi(param.second));
            }else if(param.first == "eeprom.return_delay_time"){
                dxl_eeproms_[i].return_delay_time = stoi(param.second);
            }else if(param.first == "eeprom.drive_mode"){
                dxl_eeproms_[i].drive_mode = stoi(param.second);
            }else if(param.first == "eeprom.operating_mode"){
                dxl_eeproms_[i].operating_mode = stoi(param.second);
            }else if(param.first == "eeprom.homing_offset"){
                dxl_eeproms_[i].homing_offset = stoi(param.second);
            }else if(param.first == "eeprom.moving_threshold"){
                dxl_eeproms_[i].moving_threshold = stoi(param.second);
            }else if(param.first == "eeprom.temperature_limit"){
                dxl_eeproms_[i].temperature_limit = stoi(param.second);
            }else if(param.first == "eeprom.max_voltage_limit"){
                dxl_eeproms_[i].max_voltage_limit = stoi(param.second);
            }else if(param.first == "eeprom.min_voltage_limit"){
                dxl_eeproms_[i].min_voltage_limit = stoi(param.second);
            }else if(param.first == "eeprom.pwm_limit"){
                dxl_eeproms_[i].pwm_limit = stoi(param.second);
            }else if(param.first == "eeprom.current_limit"){    
                dxl_eeproms_[i].current_limit = stoi(param.second);
            }else if(param.first == "eeprom.velocity_limit"){
                dxl_eeproms_[i].velocity_limit = stoi(param.second);
            }else if(param.first == "eeprom.max_position_limit"){
                dxl_eeproms_[i].max_position_limit = stoi(param.second);
            }else if(param.first == "eeprom.min_position_limit"){
                dxl_eeproms_[i].min_position_limit = stoi(param.second);
            }else if(param.first == "eeprom.shutdown"){
                dxl_eeproms_[i].shutdown = stoi(param.second);
            }
        }
    }

    if(joint_names.size() != (max_id_ - min_id_ + 1)){
        RCLCPP_FATAL(logger_, "(size of joints) and (max_id_ - min_id_ + 1) is not matched. (size of joints): %d, (max_id_ - min_id_ + 1): %d", (int)joint_names.size(), (int)(max_id_ - min_id_ + 1));
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto scan_error_id = handler_->scan(min_id_, max_id_, joint_names);
    if(scan_error_id == DXL_HANDLER_SUCESS){
        RCLCPP_INFO(logger_, "Succeeded to scan Dynamixel");
    }else{
        RCLCPP_FATAL(logger_, "Failed to scan Dynamixel. (ID: %d) is not found", scan_error_id);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // mimic joint
    for(auto &joint : info_.joints){
        if(joint.parameters.find("mimic") == joint.parameters.end())//mimic でない
            continue;
        string mimic_joint_name = joint.parameters.at("mimic");//元のジョイント名
        double multiplier = joint.parameters.find("multiplier") != joint.parameters.end() ? stod(joint.parameters.at("multiplier")) : 1.0;
        for(auto &dxl: handler_->dxls){
            if(dxl.joint_name == mimic_joint_name){
                MimicInfo mimic_info;
                mimic_info.joint_name = joint.name;
                mimic_info.multiplier = multiplier;
                mimic_info.original_dxl_ptr = &dxl;
                mimic_infos_.push_back(mimic_info);
                RCLCPP_INFO(logger_, "Mimic joint %s -> %s (multiplier: %.2f)", mimic_joint_name.c_str(), joint.name.c_str(), multiplier);
                break;
            }
        }
    }

    // 初期制限
    for (int i = 0; i< handler_->dxls.size(); ++i){
        auto &dxl = handler_->dxls[i];
        dxl.profile_acc = default_profile_acceleration_; // プロファイル加速度
        dxl.profile_vel = default_profile_velocity_; // プロファイル速度
        dxl.position_kp = joint_position_kp[i];
        dxl.position_ki = joint_position_ki[i];
        dxl.position_kd = joint_position_kd[i];
        dxl.limit_pos_min = joint_position_mins[i];//位置制限
        dxl.limit_pos_max = joint_position_maxs[i];//位置制限
        dxl.multiplier = joint_multipliers[i]; // 反転などの倍率
        dxl.ref_curr = 1.0;  // 電流は最大に設定
        RCLCPP_INFO(logger_, "  Joint %s (ID: %d, Model: %d) Position limit: [%.2f, %.2f], Current %.2f", dxl.joint_name.c_str(), dxl.id, dxl.model_number, dxl.limit_pos_min, dxl.limit_pos_max, dxl.ref_curr);
    }
    handler_->dxls[6].ref_curr = 0.1;

    // 初期位置
    handler_->dxls[0].ref_pos = 0;
    handler_->dxls[1].ref_pos = -1.10;
    handler_->dxls[2].ref_pos = 1.87;
    handler_->dxls[3].ref_pos = 0;
    handler_->dxls[4].ref_pos = 0.98;
    handler_->dxls[5].ref_pos = 0;
    handler_->dxls[6].ref_pos = 0;

    
    // EEPROM 書き込み
    RCLCPP_INFO(logger_, "Check and Write EEPROM if needed");
    auto write_map = handler_->set_eeprom(dxl_eeproms_);
    for(auto &write_data: write_map){
        for(auto &data : write_data.second){
            RCLCPP_INFO(logger_, "Write EEPROM: Joint ID %d, %s = %d", write_data.first, data.first.c_str(), (int)data.second);
        }
    }  

    // エラーチェック
    check_hardware_error();

    // ROS 2
    node_ = rclcpp::Node::make_shared("topoarm_hardware");
    node_->create_subscription<topoarm_msgs::msg::DynamixelCommandEffort>(
        "topoarm/effort_commands", 10, std::bind(&ToPoArmSystemHardware::effort_command_callback, this, std::placeholders::_1)
    );
    
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ToPoArmSystemHardware::check_hardware_error, this)
    );

    RCLCPP_INFO(logger_, "Succeeded to initialize Dynamixel joints");
    return hardware_interface::CallbackReturn::SUCCESS;
}

vector<hardware_interface::StateInterface>
ToPoArmSystemHardware::export_state_interfaces() {
    vector<hardware_interface::StateInterface> state_interfaces;
    for (auto& dxl : handler_->dxls) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                dxl.joint_name, hardware_interface::HW_IF_POSITION, &dxl.state_pos));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                dxl.joint_name, hardware_interface::HW_IF_VELOCITY, &dxl.state_vel));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                dxl.joint_name, hardware_interface::HW_IF_EFFORT, &dxl.state_curr));
    }
    for(auto &mimic_info: mimic_infos_){
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                mimic_info.joint_name, hardware_interface::HW_IF_POSITION, &mimic_info.state_pos));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                mimic_info.joint_name, hardware_interface::HW_IF_VELOCITY, &mimic_info.state_vel));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                mimic_info.joint_name, hardware_interface::HW_IF_EFFORT, &mimic_info.state_curr));
    }
    return state_interfaces;
}

vector<hardware_interface::CommandInterface>
ToPoArmSystemHardware::export_command_interfaces() {
    vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto& dxl : handler_->dxls) {
        command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            dxl.joint_name, hardware_interface::HW_IF_POSITION, &dxl.ref_pos));
    }
    for(auto &mimic_info: mimic_infos_){
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                mimic_info.joint_name, hardware_interface::HW_IF_POSITION, &mimic_info.ref_pos));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ToPoArmSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!handler_->set_torque(torque_)) {
        RCLCPP_ERROR(logger_, "Failed to set torque");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!handler_->set_joints_profile_acceleration_velocity()) {
        RCLCPP_ERROR(logger_, "Failed to set joints profile acceleration and velocity");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!handler_->set_joints_position_kpid()){
        RCLCPP_ERROR(logger_, "Failed to set joints position kp, ki and kd");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "System started");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ToPoArmSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "System stopped");
    if(!torque_retention_){
        handler_->set_torque(false);
        RCLCPP_INFO(logger_, "Joints torque OFF");
    }
    RCLCPP_INFO(logger_, "Port closed");
    handler_->close_port();
    node_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ToPoArmSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    RCLCPP_INFO_ONCE(logger_, "Start to read joints states");
    // auto start = std::chrono::steady_clock::now();
    if (!handler_->read_all()) {
        // RCLCPP_WARN(logger_, "Failed to read");
    }
    for(auto &mimic_info: mimic_infos_){
        mimic_info.state_pos = mimic_info.original_dxl_ptr->state_pos * mimic_info.multiplier;
        mimic_info.state_vel = mimic_info.original_dxl_ptr->state_vel * mimic_info.multiplier;
        mimic_info.state_curr = mimic_info.original_dxl_ptr->state_curr * mimic_info.multiplier;
    }
    // auto end = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // RCLCPP_INFO(logger_, "Read all joints took %ld ms", duration);
    rclcpp::spin_some(node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ToPoArmSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    RCLCPP_INFO_ONCE(logger_, "Start to write joints commands");
    // auto start = std::chrono::steady_clock::now();
    if(!handler_->set_joints_current()){
        RCLCPP_ERROR(logger_, "Failed to set joints current");
    }
    if(!handler_->set_joints_position()){
        RCLCPP_ERROR(logger_, "Failed to set joints position");
    }
    // auto end = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // RCLCPP_INFO(logger_, "Write all joints took %ld ms", duration);
    return hardware_interface::return_type::OK;
}
void ToPoArmSystemHardware::effort_command_callback(const topoarm_msgs::msg::DynamixelCommandEffort::SharedPtr msg) {
    size_t effort_size = msg->current.size();
    size_t dxl_size = msg->id_list.size();
    if (effort_size != dxl_size) {
        RCLCPP_ERROR(logger_, "Size of current command is not matched");
        return;
    }
    for (int i = 0; i < dxl_size; ++i){
        for(auto &dxl: handler_->dxls){
            if(dxl.id == msg->id_list[i]){
                dxl.ref_curr = msg->current[i];
                break;
            }
        }
    }
}
void ToPoArmSystemHardware::check_hardware_error(){
    vector<uint8_t> error_ids, torque_off_ids;
    if(!handler_->read_error_id(error_ids, torque_off_ids)){
        RCLCPP_ERROR(logger_, "Failed to check error");
    }else{
        if(!torque_off_ids.empty() || !error_ids.empty()){
            for(auto &id : error_ids){
                RCLCPP_ERROR(logger_, "Hardware error detected (ID: %d)", id);
                handler_->reboot(id);
                RCLCPP_ERROR(logger_, "Rebooted (ID: %d)", id);
            }
            if(torque_){
                for(auto id: torque_off_ids){
                    handler_->set_torque(true);
                    RCLCPP_ERROR(logger_, "Torque was turned off (ID: %d). Turned on again.", id);
                }
            }
            handler_->set_joints_profile_acceleration_velocity();
            handler_->set_joints_position_kpid();
        }
    }
}
}  // namespace topoarm_hardware
}  // namespace fuzzrobo

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    fuzzrobo::topoarm_hardware::ToPoArmSystemHardware,
    hardware_interface::SystemInterface)
