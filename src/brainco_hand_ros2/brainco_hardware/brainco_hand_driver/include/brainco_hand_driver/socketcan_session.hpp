// Copyright (c) 2025 BrainCo
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

#include "brainco_hand_driver/transport_session_base.hpp"
#include "stark-sdk.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

namespace brainco_hand_driver
{

// CAN FD transport session implemented on top of the Linux SocketCAN stack.
//
// The Stark SDK is transport agnostic: it builds the Revo2 protocol frames and
// hands them to the TX/RX callbacks registered through set_can_tx_callback() /
// set_can_rx_callback(). This session provides those callbacks backed by a raw
// AF_CAN socket, so no vendor USB-CAN library is involved and the bus can be
// inspected with the standard tools (candump, ip -s link).
//
// Bit timing is NOT configured here — it belongs to the network interface and
// must be set beforehand, e.g.:
//   ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
//
// Several sessions may be open at once — one per hand. The SDK's CAN callbacks
// are process-wide function pointers with no user-data argument, so they cannot
// be bound to an instance; instead every open session registers itself under its
// slave id and the static callbacks dispatch on the slave_id the SDK passes.
// That is what lets a bimanual setup run two hands, whether they sit on separate
// interfaces or share one bus.
class SocketCanSession final : public SessionBase
{
public:
  explicit SocketCanSession(BraincoHandApi::DriverConfig & config);
  ~SocketCanSession() override;

  SocketCanSession(const SocketCanSession &) = delete;
  SocketCanSession & operator=(const SocketCanSession &) = delete;

  bool open() override;
  void close() override;
  [[nodiscard]] bool is_open() const override;
  [[nodiscard]] std::optional<BraincoHandApi::ConnectionInfo> connection_info() const override;

private:
  using SocketCanHandlePtr = std::unique_ptr<DeviceHandler, decltype(&free_device_handler)>;

  // Sentinel for "no socket owned"; matches the value returned by a failed socket().
  static constexpr int kInvalidSocketFd{-1};

  static int32_t tx_callback(
    uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len);
  static int32_t rx_callback(
    uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out);
  static int32_t noop_tx_callback(uint8_t, uint32_t, const uint8_t *, uintptr_t);
  static int32_t noop_rx_callback(uint8_t, uint32_t *, uint8_t *, uintptr_t *);

  int32_t handle_tx(uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len);
  int32_t handle_rx(
    uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out);

  // Creates, configures and binds the raw CAN socket described by config_.socketcan.
  // Returns false and logs the reason on any failure; leaves no socket behind.
  bool open_socket();
  void close_socket();

  SocketCanHandlePtr handle_;
  int socket_fd_{kInvalidSocketFd};
  mutable std::mutex socket_mutex_{};

  // Edge-triggered log gates: TX/RX run at the control rate, so faults are
  // logged on transition only instead of on every cycle.
  // Times write() hit a full TX queue and had to be retried. Non-fatal, but the
  // count is the clearest indicator that the bus is running near capacity.
  std::size_t tx_backpressure_events_{0};

  bool tx_fault_logged_{false};
  bool rx_fault_logged_{false};
  bool bus_error_logged_{false};

  // Slave id this session registered under; kept so close() can deregister even
  // if the config was mutated in the meantime.
  uint8_t registered_slave_id_{0};
  bool registered_{false};

  /// Resolves the session the SDK is currently talking to.
  /// Falls back to the sole open session when the slave id does not match one —
  /// the vendor CAN FD path ignores slave_id entirely, so it cannot be assumed
  /// to be populated on every SDK build.
  static SocketCanSession * lookup_session(uint8_t slave_id);

  static std::mutex registry_mutex_;
  static std::map<uint8_t, SocketCanSession *> sessions_by_slave_;
};

}  // namespace brainco_hand_driver
