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

#ifndef DYNAMIXEL_HANDLER_HPP_
#define DYNAMIXEL_HANDLER_HPP_

#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <lib_dynamixel/src/dynamixel_communicator.h>

using namespace std;

#define DXL_HANDLER_SUCESS (-1)

struct Dynamixel{
    string joint_name;
    uint8_t id;
    uint16_t model_number;
    double multiplier;
    double ref_pos;
    double ref_vel;
    double ref_curr;
    double state_pos;
    double state_vel;
    double state_curr;
    double profile_acc;
    double profile_vel;
    int position_kp;
    int position_ki;
    int position_kd;
    double limit_pos_min;
    double limit_pos_max;
};

struct DynamixelEEPROM{
    uint8_t return_delay_time;
    uint8_t drive_mode;
    uint8_t operating_mode;
    uint8_t secondary_id;
    int32_t homing_offset;
    uint32_t moving_threshold;
    uint8_t temperature_limit;
    uint16_t max_voltage_limit;
    uint16_t min_voltage_limit;
    uint16_t pwm_limit;
    uint16_t current_limit;
    uint32_t velocity_limit;
    uint32_t max_position_limit;
    uint32_t min_position_limit;
    int16_t shutdown;
};

namespace fuzzrobo {
namespace topoarm_hardware {
class DynamixelHandler {
   public:
    vector<Dynamixel> dxls;
    vector<uint8_t> dxl_ids;
    explicit DynamixelHandler(const string &usb_port, const int baud_rate, const int latency_timer);
    virtual ~DynamixelHandler();
    string port_name() { return dxl_comm_.port_name(); }
    bool open_port();
    bool close_port();
    void reboot(const uint8_t &id);
    int scan(const uint8_t &min_id, const uint8_t &max_id, const vector<string> &joint_names);
    int set_torque(bool on);
    bool read_error_id(vector<uint8_t> &error_ids, vector<uint8_t> &torque_off_ids);
    bool read_all();
    bool write_all();
    bool set_joints_current();
    bool set_joints_position();
    bool set_joints_profile_acceleration_velocity();
    double dxl_current_max(uint16_t model_num) const {
        if (dynamixel_series(model_num) == SERIES_P) return 5.5;
        else // dynamixel_series(model_num) == SERIES_X || SERIES_PRO
        switch (model_num) {
            // Proシリーズ
            case MODEL_H42_020_S300:
            case MODEL_M42_010_S260: 
            case MODEL_L42_010_S300:
            case MODEL_H54_200_S500:
            case MODEL_H54_100_S500:
            case MODEL_M54_060_S250:
            case MODEL_M54_040_S250:
            case MODEL_L54_050_S290:
            case MODEL_L54_050_S500:
            case MODEL_L54_030_S400:
            case MODEL_L54_030_S500:  return 5.5;
            // Xシリーズ
            case MODEL_XL330_M077:
            case MODEL_XL330_M288:    return 1.5;
            case MODEL_XC330_M181:
            case MODEL_XC330_M288:    return 1.8;
            case MODEL_XC330_T181:
            case MODEL_XC330_T288:    return 0.8;
            case MODEL_XM430_W210:
            case MODEL_XM430_W350:    return 2.3;
            case MODEL_XM540_W150:
            case MODEL_XM540_W270:    return 4.4;
            default:                  return 4.4; /*A*/
        } 
    }
    bool set_joints_position_kpid();
    map<uint8_t, vector<pair<string, int64_t>>> set_eeprom(const vector<DynamixelEEPROM> &eeprom);

   private:
    DynamixelCommunicator dxl_comm_;
};
}  // namespace topoarm_hardware
}  // namespace fuzzrobo
#endif  // DYNAMIXEL_HANDLER_HPP_
