// Copyright 2025 FuzzRoBo CO., LTD.
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
// Author: Nagashima

#include <topoarm_hardware/dynamixel_handler.hpp>

namespace fuzzrobo {
namespace topoarm_hardware {
DynamixelHandler::DynamixelHandler(const std::string& usb_port, const int baud_rate, const int latency_timer) : dxl_comm_(usb_port.c_str(), baud_rate, latency_timer){
}

DynamixelHandler::~DynamixelHandler() {
}

bool DynamixelHandler::open_port() {
    return dxl_comm_.OpenPort();
}
bool DynamixelHandler::close_port() {
    return dxl_comm_.ClosePort();
}
int DynamixelHandler::scan(const uint8_t &min_id, const uint8_t &max_id, const vector<string> &joint_names){
    for (int i = 0; i < joint_names.size(); ++i) {
        uint8_t id = min_id + i;
        if(!dxl_comm_.Ping(id)){
            return id;
        }
        int64_t model_number = dxl_comm_.Read(AddrCommon::model_number, id);
        if (model_number == 0) {
            return id;
        }
        Dynamixel dxl;
        dxl.id = id;
        dxl.joint_name = joint_names[i];
        dxl.model_number = model_number;
        // ROS2 Controllerからの入出力
        dxls.push_back(dxl);
        // 認識してる ID リスト
        dxl_ids.push_back(id);
    }
    return DXL_HANDLER_SUCESS;
}
int DynamixelHandler::set_torque(bool on) {
    int64_t data = on ? TORQUE_ENABLE : TORQUE_DISABLE;
    for (auto id : dxl_ids) {
        if (!dxl_comm_.Write(AddrX::torque_enable, id, data)){
            return id;
        }
    }
    return DXL_HANDLER_SUCESS;
}
void DynamixelHandler::reboot(const uint8_t &id) {
    dxl_comm_.Reboot(id);
}
bool DynamixelHandler::read_error_id(vector<uint8_t> &error_ids, vector<uint8_t> &torque_off_ids){
    vector<DynamixelAddress> addr_list = {
        AddrX::torque_enable,
        AddrX::led,
        AddrX::status_return_level,
        AddrX::registered_instruction,
        AddrX::hardware_error_status
    };
    auto results = dxl_comm_.SyncRead(addr_list, dxl_ids);
    if(results.size() != dxl_ids.size())
        return false;
    for(auto &result: results){
        if(result.second.size() != addr_list.size()){
            return false;
        }
        if(result.second[0] != 1){ // torque_off
            torque_off_ids.push_back(result.first);
        }
        if(result.second[4] != 0){ // hardware_error
            error_ids.push_back(result.first);
        }
    }
    return true;
}

bool DynamixelHandler::read_all() {
    vector<DynamixelAddress> addr_list = {
        AddrX::present_position,
        AddrX::present_velocity,
        AddrX::present_current,
    };
    auto results = dxl_comm_.SyncRead(addr_list, dxl_ids);
    if(results.size() != dxl_ids.size())
        return false;
    for(auto &result: results){
        auto id = result.first;
        auto data_vec = result.second;
        if(data_vec.size() != addr_list.size())
            continue;
        for(auto &dxl: dxls){
            if(dxl.id == id){
                dxl.state_pos = addr_list[0].pulse2val(data_vec[0], dxl.model_number);
                dxl.state_vel = addr_list[1].pulse2val(data_vec[1], dxl.model_number);
                dxl.state_curr = addr_list[2].pulse2val(data_vec[2], dxl.model_number) * 0.001; // mA -> A

                dxl.state_pos *= dxl.multiplier;
                dxl.state_vel *= dxl.multiplier;
                // dxl.state_curr *= dxl.multiplier;
                break;
            }
        }
    }
    return true;
}

bool DynamixelHandler::set_joints_current(){
    vector<int64_t> current_vec;
    for(auto &dxl: dxls){
        double ref_curr_max = dxl_current_max(dxl.model_number);
        double ref_curr = std::clamp(dxl.ref_curr, -ref_curr_max, ref_curr_max);
        current_vec.push_back(AddrX::goal_current.val2pulse(ref_curr * 1000, dxl.model_number)); // A -> mA
    }
    return dxl_comm_.SyncWrite(AddrX::goal_current, dxl_ids, current_vec);
}

bool DynamixelHandler::set_joints_position(){
    vector<int64_t> position_vec;
    for(auto &dxl: dxls){
        double ref_pos = std::clamp(dxl.ref_pos, dxl.limit_pos_min, dxl.limit_pos_max);
        position_vec.push_back(AddrX::goal_position.val2pulse(ref_pos/dxl.multiplier, dxl.model_number));
    }
    return dxl_comm_.SyncWrite(AddrX::goal_position, dxl_ids, position_vec);
}

bool DynamixelHandler::set_joints_profile_acceleration_velocity(){
    vector<DynamixelAddress> addr_list = {
        AddrX::profile_acceleration,
        AddrX::profile_velocity
    };
    vector<vector<int64_t>> vec;
    vec.reserve(dxls.size());
    for (auto& dxl : dxls) {
        vector<int64_t> v(2);
        v[0] = AddrX::profile_acceleration.val2pulse(dxl.profile_acc, dxl.model_number);
        v[1] = AddrX::profile_velocity.val2pulse(dxl.profile_vel, dxl.model_number);
        vec.emplace_back(v);
    }
    return dxl_comm_.SyncWrite(addr_list, dxl_ids, vec);
}
bool DynamixelHandler::set_joints_position_kpid(){
    vector<DynamixelAddress> addr_list = {
        AddrX::position_d_gain,
        AddrX::position_i_gain,
        AddrX::position_p_gain
    };
    vector<vector<int64_t>> vec;
    vec.reserve(dxls.size());
    for (auto& dxl : dxls) {
        vector<int64_t> v(3);
        v[0] = std::clamp(dxl.position_kd, 0, 16383);
        v[1] = std::clamp(dxl.position_ki, 0, 16383);
        v[2] = std::clamp(dxl.position_kp, 0, 16383);
        vec.emplace_back(v);
    }
    return dxl_comm_.SyncWrite(addr_list, dxl_ids, vec);
}
map<uint8_t, vector<pair<string, int64_t>>> DynamixelHandler::set_eeprom(const vector<DynamixelEEPROM> &dxl_eeproms){
    vector<DynamixelAddress> addr_list = {
        AddrX::return_delay_time,
        AddrX::drive_mode,
        AddrX::operating_mode,
        AddrX::homing_offset,
        AddrX::moving_threshold,
        AddrX::temperature_limit,
        AddrX::max_voltage_limit,
        AddrX::min_voltage_limit,
        AddrX::pwm_limit,
        AddrX::current_limit,
        AddrX::velocity_limit,
        AddrX::max_position_limit,
        AddrX::min_position_limit,
        AddrX::shutdown
    };
    map<uint8_t, vector<pair<string, int64_t>>> write_map;
    for(int i=0; i<dxls.size(); ++i){
        uint8_t id = dxls[i].id;
        auto &eeprom = dxl_eeproms[i];
        vector<pair<string, int64_t>> write_vec{
            {"return_delay_time", eeprom.return_delay_time},
            {"drive_mode", eeprom.drive_mode},
            {"operating_mode", eeprom.operating_mode},
            {"homing_offset", eeprom.homing_offset},
            {"moving_threshold", eeprom.moving_threshold},
            {"temperature_limit", eeprom.temperature_limit},
            {"max_voltage_limit", eeprom.max_voltage_limit},
            {"min_voltage_limit", eeprom.min_voltage_limit},
            {"pwm_limit", eeprom.pwm_limit},
            {"current_limit", eeprom.current_limit},
            {"velocity_limit", eeprom.velocity_limit},
            {"max_position_limit", eeprom.max_position_limit},
            {"min_position_limit", eeprom.min_position_limit},
            {"shutdown", eeprom.shutdown}
        };
        vector<pair<string, int64_t>> write_addr_name;
        int64_t prev_torque = dxl_comm_.Read(AddrX::torque_enable, id);
        int64_t now_torque = prev_torque;
        for(int i=0; i<addr_list.size(); ++i){
            auto data = dxl_comm_.Read(addr_list[i], id);
            if(data != write_vec[i].second){
                if(now_torque == 1){
                    dxl_comm_.Write(AddrX::torque_enable, id, 0);// torque onだと変更できない
                    now_torque = 0;
                }
                dxl_comm_.Write(addr_list[i], id, write_vec[i].second);
                write_addr_name.push_back(write_vec[i]);
            }
        }
        if(now_torque != prev_torque)
            dxl_comm_.Write(AddrX::torque_enable, id, prev_torque);
        write_map.emplace(id, write_addr_name);
    }
    return write_map;
}
}  // namespace topoarm_hardware
}  // namespace fuzzrobo
