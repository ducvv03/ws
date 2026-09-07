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

#include <chrono>
#include <iostream>
#include <limits>
#include <optional>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <stdexcept>
#include <string>

namespace openarm::damiao_motor {

// Constructor
Motor::Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id)
    : send_can_id_(send_can_id),
      recv_can_id_(recv_can_id),
      motor_type_(motor_type),
      state_q_(0.0),
      state_dq_(0.0),
      state_tau_(0.0),
      state_tmos_(0),
      state_trotor_(0),
      state_status_(MotorStatus::DISABLED),
      state_raw_status_byte_(0),
      state_last_fault_(std::nullopt),
      state_last_recv_time_(),
      state_recv_count_(0) {}

// Parameter methods
// TODO: storing temp params in motor object might not be a good idea
// also -1 is not a good default value, consider using a different value
double Motor::get_param(int RID) const {
    auto it = temp_param_dict_.find(RID);
    return (it != temp_param_dict_.end()) ? it->second : -1;
}

void Motor::set_temp_param(int RID, double val) { temp_param_dict_[RID] = val; }

// State update methods
void Motor::update_state(double q, double dq, double tau, int tmos, int trotor, MotorStatus status,
                         uint8_t raw_status_byte) {
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
    state_tmos_ = tmos;
    state_trotor_ = trotor;
    state_raw_status_byte_ = raw_status_byte;

    // This runs once per received frame, so log on the status *edge* only:
    // logging every frame would wreck timing in a control loop.
    if (state_recv_count_ == 0 || status != state_status_) {
        std::cerr << (is_fault_status(status) ? "ERROR: " : "INFO: ") << "motor 0x" << std::hex
                  << send_can_id_ << std::dec << " status " << motor_status_to_string(state_status_)
                  << " -> " << motor_status_to_string(status) << " (D[0]=0x" << std::hex
                  << static_cast<int>(raw_status_byte) << std::dec << ")" << std::endl;
    }
    state_status_ = status;
    if (is_fault_status(status)) {
        state_last_fault_ = status;
    }

    state_last_recv_time_ = std::chrono::steady_clock::now();
    ++state_recv_count_;
}

double Motor::get_time_since_last_state_s() const {
    if (state_recv_count_ == 0) {
        // Never heard from: infinitely old rather than "0 s ago".
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - state_last_recv_time_)
        .count();
}

bool Motor::is_state_stale(double timeout_s) const {
    if (!(timeout_s > 0.0)) {
        // A non-positive or NaN window would mark every motor stale.
        std::cerr << "WARNING: motor 0x" << std::hex << send_can_id_ << std::dec
                  << " invalid stale timeout " << timeout_s << " s, falling back to "
                  << DEFAULT_STATE_TIMEOUT_S << " s" << std::endl;
        timeout_s = DEFAULT_STATE_TIMEOUT_S;
    }
    return get_time_since_last_state_s() > timeout_s;
}

void Motor::set_state_tmos(int tmos) { state_tmos_ = tmos; }

void Motor::set_state_trotor(int trotor) { state_trotor_ = trotor; }

// Static methods
LimitParam Motor::get_limit_param(MotorType motor_type) {
    size_t index = static_cast<size_t>(motor_type);
    if (index >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid motor type: " +
                                    std::to_string(static_cast<int>(motor_type)));
    }
    return MOTOR_LIMIT_PARAMS[index];
}

}  // namespace openarm::damiao_motor
