// Copyright 2025 Enactic, Inc.
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

#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>

#include "dm_motor_constants.hpp"

namespace openarm::damiao_motor {
class Motor {
    friend class DMCANDevice;  // Allow MotorDeviceCan to access protected
                               // members
    friend class DMControl;

public:
    // Constructor
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // State getters
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }

    // Status reported by the motor itself in D[0] of every state frame.
    MotorStatus get_status() const { return state_status_; }
    uint8_t get_raw_status_byte() const { return state_raw_status_byte_; }
    bool has_fault() const { return is_fault_status(state_status_); }

    // Latched fault: the motor reverts to DISABLED once it trips, which loses
    // the reason it shut down. This keeps it until explicitly cleared.
    std::optional<MotorStatus> get_last_fault() const { return state_last_fault_; }
    void clear_last_fault() { state_last_fault_.reset(); }

    // Feedback freshness. Position/velocity/torque keep their last value
    // forever, so freshness is the only way to tell "motor sitting at 0 rad"
    // apart from "motor stopped answering".
    bool has_received_state() const { return state_recv_count_ > 0; }
    uint64_t get_state_recv_count() const { return state_recv_count_; }
    double get_time_since_last_state_s() const;
    bool is_state_stale(double timeout_s = DEFAULT_STATE_TIMEOUT_S) const;

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // Enable status getters. Derived from the motor's own status nibble, so it
    // reflects the hardware rather than what was last commanded.
    bool is_enabled() const { return state_status_ == MotorStatus::ENABLED; }

    // Parameter methods
    double get_param(int RID) const;

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(double q, double dq, double tau, int tmos, int trotor, MotorStatus status,
                      uint8_t raw_status_byte);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_temp_param(int RID, double val);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Current state
    double state_q_, state_dq_, state_tau_;
    int state_tmos_, state_trotor_;
    MotorStatus state_status_;
    uint8_t state_raw_status_byte_;
    std::optional<MotorStatus> state_last_fault_;

    // Feedback bookkeeping, used for staleness detection
    std::chrono::steady_clock::time_point state_last_recv_time_;
    uint64_t state_recv_count_;

    // Parameter storage
    std::map<int, double> temp_param_dict_;
};
}  // namespace openarm::damiao_motor
