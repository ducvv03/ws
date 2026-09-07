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

#include <robot_state.hpp>
#include <iostream>
#include <vector>

// Represents the state of a single joint
// struct JointState {
//     double position = 0.0;
//     double velocity = 0.0;
//     double effort   = 0.0;
// };

// Represents the state of a single motor (raw values)
struct MotorState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

// Abstract base class for converting between motor and joint states
class MotorJointConverter {
public:
    virtual ~MotorJointConverter() = default;

    // MotorState vector → JointState vector
    virtual std::vector<JointState> motor_to_joint(
        const std::vector<MotorState>& motor_states) const = 0;

    // JointState vector → MotorState vector
    virtual std::vector<MotorState> joint_to_motor(
        const std::vector<JointState>& joint_states) const = 0;

    virtual size_t get_joint_count() const = 0;
};

// assume motor num equals to joint num
class OpenArmJointConverter : public MotorJointConverter {
public:
    explicit OpenArmJointConverter(size_t joint_count) : joint_count_(joint_count) {
        std::cout << "OpenArm joint converter joinit_count is : " << joint_count << std::endl;
    }

    std::vector<JointState> motor_to_joint(const std::vector<MotorState>& m) const override {
        // std::cout << "joint num conv : " << m.size() << std::endl;

        std::vector<JointState> j(m.size());
        for (size_t i = 0; i < m.size(); ++i) {
            j[i] = {m[i].position, m[i].velocity, m[i].effort};
        }

        return j;
    }

    std::vector<MotorState> joint_to_motor(const std::vector<JointState>& j) const override {
        std::vector<MotorState> m(j.size());
        for (size_t i = 0; i < j.size(); ++i) m[i] = {j[i].position, j[i].velocity, j[i].effort};
        return m;
    }

    // Merge the 3 raw state vectors (position/velocity/effort) coming from the
    // hardware into a single JointState vector. Returns an empty vector when any
    // pointer is null or the sizes do not match.
    std::vector<JointState> states_to_joint(const std::vector<double>* pos_states,
                                            const std::vector<double>* vel_states,
                                            const std::vector<double>* tau_states) const {
        std::vector<JointState> joint_arm_states;

        // Null check: bail out (empty) if any pointer is null.
        if (pos_states == nullptr || vel_states == nullptr || tau_states == nullptr) {
            std::cerr << "states_to_joint: null pointer input" << std::endl;
            return joint_arm_states;
        }

        // Size check: all 3 vectors must have the same length to zip by index.
        const size_t n = pos_states->size();
        if (vel_states->size() != n || tau_states->size() != n) {
            std::cerr << "states_to_joint: size mismatch (pos=" << n
                      << " vel=" << vel_states->size() << " tau=" << tau_states->size() << ")"
                      << std::endl;
            return joint_arm_states;
        }

        joint_arm_states.resize(n);
        for (size_t i = 0; i < n; ++i) {
            joint_arm_states[i].position = (*pos_states)[i];
            joint_arm_states[i].velocity = (*vel_states)[i];
            joint_arm_states[i].effort = (*tau_states)[i];
        }
        return joint_arm_states;
    }

    size_t get_joint_count() const override { return joint_count_; }

private:
    size_t joint_count_;
};

// assume motor num equals to joint num
class OpenArmJGripperJointConverter : public MotorJointConverter {
public:
    explicit OpenArmJGripperJointConverter(size_t joint_count) : joint_count_(joint_count) {
        std::cout << "Gripper joint converter joint_count is : " << joint_count << std::endl;
    }

    std::vector<JointState> motor_to_joint(const std::vector<MotorState>& m) const override {
        std::vector<JointState> j(m.size());
        for (size_t i = 0; i < m.size(); ++i) {
            j[i] = {m[i].position, m[i].velocity, m[i].effort};
        }
        return j;
    }

    std::vector<MotorState> joint_to_motor(const std::vector<JointState>& j) const override {
        std::vector<MotorState> m(j.size());
        for (size_t i = 0; i < j.size(); ++i) {
            m[i] = {j[i].position, j[i].velocity, j[i].effort};
        }
        return m;
    }

    size_t get_joint_count() const override { return joint_count_; }

private:
    size_t joint_count_;
};