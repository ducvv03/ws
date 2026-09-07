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

#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::damiao_motor {
enum class MotorType : uint8_t {
    DM3507 = 0,
    DM4310 = 1,
    DM4310_48V = 2,
    DM4340 = 3,
    DM4340_48V = 4,
    DM6006 = 5,
    DM8006 = 6,
    DM8009 = 7,
    DM10010L = 8,
    DM10010 = 9,
    DMH3510 = 10,
    DMH6215 = 11,
    DMG6220 = 12,
    COUNT = 13
};

enum class ControlMode : uint8_t { MIT = 1, POS_VEL = 2, VEL = 3, POS_FORCE = 4 };

// --- Motor feedback status ------------------------------------------------
//
// D[0] of a state frame packs two nibbles (DaMiao motor manual):
//   bit0-3 : ESC (slave) CAN ID that produced the frame
//   bit4-7 : motor status / error code
inline constexpr uint8_t FEEDBACK_NIBBLE_MASK = 0x0F;
inline constexpr uint8_t FEEDBACK_STATUS_NIBBLE_SHIFT = 4;

// Minimum payload of a state / param feedback frame (classic CAN).
inline constexpr std::size_t FEEDBACK_FRAME_MIN_BYTES = 8;

// A motor whose last state frame is older than this is treated as stale.
inline constexpr double DEFAULT_STATE_TIMEOUT_S = 0.1;

enum class MotorStatus : uint8_t {
    DISABLED = 0x0,
    ENABLED = 0x1,
    OVERVOLTAGE = 0x8,
    UNDERVOLTAGE = 0x9,
    OVERCURRENT = 0xA,
    MOS_OVERTEMPERATURE = 0xB,
    COIL_OVERTEMPERATURE = 0xC,
    COMMUNICATION_LOST = 0xD,
    OVERLOAD = 0xE,
    // Not defined by the manual: any code this build does not recognize.
    UNKNOWN = 0xF
};

// Map a raw status nibble onto MotorStatus, folding undocumented codes to
// UNKNOWN so an unexpected firmware value can never be read as "healthy".
inline constexpr MotorStatus to_motor_status(uint8_t status_nibble) {
    switch (status_nibble) {
        case static_cast<uint8_t>(MotorStatus::DISABLED):
            return MotorStatus::DISABLED;
        case static_cast<uint8_t>(MotorStatus::ENABLED):
            return MotorStatus::ENABLED;
        case static_cast<uint8_t>(MotorStatus::OVERVOLTAGE):
            return MotorStatus::OVERVOLTAGE;
        case static_cast<uint8_t>(MotorStatus::UNDERVOLTAGE):
            return MotorStatus::UNDERVOLTAGE;
        case static_cast<uint8_t>(MotorStatus::OVERCURRENT):
            return MotorStatus::OVERCURRENT;
        case static_cast<uint8_t>(MotorStatus::MOS_OVERTEMPERATURE):
            return MotorStatus::MOS_OVERTEMPERATURE;
        case static_cast<uint8_t>(MotorStatus::COIL_OVERTEMPERATURE):
            return MotorStatus::COIL_OVERTEMPERATURE;
        case static_cast<uint8_t>(MotorStatus::COMMUNICATION_LOST):
            return MotorStatus::COMMUNICATION_LOST;
        case static_cast<uint8_t>(MotorStatus::OVERLOAD):
            return MotorStatus::OVERLOAD;
        default:
            return MotorStatus::UNKNOWN;
    }
}

// Anything that is neither DISABLED nor ENABLED is a motor fault: the motor
// has dropped torque output and needs a clear-error command to come back.
inline constexpr bool is_fault_status(MotorStatus status) {
    return status != MotorStatus::DISABLED && status != MotorStatus::ENABLED;
}

inline constexpr const char* motor_status_to_string(MotorStatus status) {
    switch (status) {
        case MotorStatus::DISABLED:
            return "DISABLED";
        case MotorStatus::ENABLED:
            return "ENABLED";
        case MotorStatus::OVERVOLTAGE:
            return "OVERVOLTAGE";
        case MotorStatus::UNDERVOLTAGE:
            return "UNDERVOLTAGE";
        case MotorStatus::OVERCURRENT:
            return "OVERCURRENT";
        case MotorStatus::MOS_OVERTEMPERATURE:
            return "MOS_OVERTEMP";
        case MotorStatus::COIL_OVERTEMPERATURE:
            return "COIL_OVERTEMP";
        case MotorStatus::COMMUNICATION_LOST:
            return "COMM_LOST";
        case MotorStatus::OVERLOAD:
            return "OVERLOAD";
        case MotorStatus::UNKNOWN:
            break;
    }
    return "UNKNOWN";
}

enum class RID : uint8_t {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
    COUNT = 82
};

// Limit parameters structure for different motor types
struct LimitParam {
    double pMax;  // Position limit (rad)
    double vMax;  // Velocity limit (rad/s)
    double tMax;  // Torque limit (Nm)
};
// Limit parameters for each motor type [pMax, vMax, tMax]
inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)>
    MOTOR_LIMIT_PARAMS = {{
        {12.5, 50, 5},    // DM3507
        {12.5, 30, 10},   // DM4310
        {12.5, 50, 10},   // DM4310_48V
        {12.5, 10, 28},   // DM4340
        {12.5, 10, 28},   // DM4340_48V
        {12.5, 45, 20},   // DM6006
        {12.5, 45, 40},   // DM8006
        {12.5, 45, 54},   // DM8009
        {12.5, 25, 200},  // DM10010L
        {12.5, 20, 200},  // DM10010
        {12.5, 280, 1},   // DMH3510
        {12.5, 45, 10},   // DMH6215
        {12.5, 45, 10}    // DMG6220
    }};
}  // namespace openarm::damiao_motor
