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

#include "brainco_hand_driver/socketcan_session.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

#include "brainco_hand_driver/logger_macros.hpp"

namespace brainco_hand_driver
{

namespace
{
// How many times to re-issue a write() that came back ENOBUFS, and how long to
// wait between attempts. The queue drains at wire rate -- roughly 48 us for an
// 8-byte CAN FD frame at 1 Mbps arbitration / 5 Mbps data -- so this backoff
// frees several slots. Worst case the whole loop costs 1 ms, comfortably inside
// the 8 ms period of a component running asynchronously at 125 Hz.
constexpr int kTxEnobufsRetries{4};
constexpr auto kTxEnobufsBackoff = std::chrono::microseconds{250};

// Interval for the back-pressure running total: a health trend, not an event.
constexpr int64_t kBackpressureSummaryThrottleMs{10000};

// Blocking limit for a single write() on the CAN socket. The SDK calls the TX
// callback from the control path, so a full TX queue must fail fast instead of
// stalling the caller.
constexpr int64_t kSendTimeoutMs{100};

constexpr int64_t kMillisecondsPerSecond{1000};
constexpr int64_t kMicrosecondsPerMillisecond{1000};

// setsockopt() flag values, named so the call sites read as intent.
constexpr int kSockOptEnable{1};
constexpr int kSockOptDisable{0};

// poll() returns 0 on timeout, which is the normal "no reply yet" outcome.
constexpr int kPollTimeout{0};

// Callback dispatch failures repeat at the control rate, so rate-limit them.
constexpr int64_t kDispatchLogThrottleMs{2000};

timeval to_timeval(int64_t milliseconds)
{
  timeval value{};
  value.tv_sec = static_cast<time_t>(milliseconds / kMillisecondsPerSecond);
  value.tv_usec =
    static_cast<suseconds_t>((milliseconds % kMillisecondsPerSecond) * kMicrosecondsPerMillisecond);
  return value;
}
}  // namespace

std::mutex SocketCanSession::registry_mutex_;
std::map<uint8_t, SocketCanSession *> SocketCanSession::sessions_by_slave_;

SocketCanSession * SocketCanSession::lookup_session(uint8_t slave_id)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);

  const auto match = sessions_by_slave_.find(slave_id);
  if (match != sessions_by_slave_.end())
  {
    return match->second;
  }

  // Single-hand setups must keep working even if the SDK passes a slave id this
  // driver never registered (0, for instance).
  if (sessions_by_slave_.size() == 1)
  {
    return sessions_by_slave_.begin()->second;
  }

  return nullptr;
}

SocketCanSession::SocketCanSession(BraincoHandApi::DriverConfig & config)
: SessionBase(config), handle_(nullptr, &free_device_handler)
{
}

SocketCanSession::~SocketCanSession() { SocketCanSession::close(); }

bool SocketCanSession::open()
{
  close();

  const uint8_t slave_id = config_.slave_id;
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    const auto clash = sessions_by_slave_.find(slave_id);
    if (clash != sessions_by_slave_.end() && clash->second != this)
    {
      BRAINCO_HAND_LOG_ERROR(
        "slave id %u is already served by an open SocketCAN session; give each hand its own "
        "slave_id (the shipped configs use 126 for left and 127 for right)",
        static_cast<unsigned>(slave_id));
      return false;
    }
  }

  if (!open_socket())
  {
    // open_socket() already logged the specific failure.
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    sessions_by_slave_[slave_id] = this;
    registered_slave_id_ = slave_id;
    registered_ = true;
  }

  // Idempotent: every session installs the same static dispatchers.
  set_can_tx_callback(&SocketCanSession::tx_callback);
  set_can_rx_callback(&SocketCanSession::rx_callback);

  DeviceHandler * raw = ::canfd_init(config_.socketcan.master_id);
  if (raw == nullptr)
  {
    BRAINCO_HAND_LOG_ERROR(
      "canfd_init failed for master_id=%u on interface '%s'",
      static_cast<unsigned>(config_.socketcan.master_id), config_.socketcan.interface.c_str());
    close();
    return false;
  }

  handle_.reset(raw);
  set_handler(raw);

  std::size_t open_sessions = 0;
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    open_sessions = sessions_by_slave_.size();
  }

  BRAINCO_HAND_LOG_INFO(
    "SocketCAN session opened on '%s' for slave %u (master_id=%u, extended_id=%s, brs=%s); "
    "%zu session(s) now open",
    config_.socketcan.interface.c_str(), static_cast<unsigned>(slave_id),
    static_cast<unsigned>(config_.socketcan.master_id),
    config_.socketcan.use_extended_id ? "true" : "false",
    config_.socketcan.enable_bitrate_switch ? "true" : "false", open_sessions);
  return true;
}

void SocketCanSession::close()
{
  bool registry_now_empty = false;
  if (registered_)
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    const auto entry = sessions_by_slave_.find(registered_slave_id_);
    if (entry != sessions_by_slave_.end() && entry->second == this)
    {
      sessions_by_slave_.erase(entry);
    }
    registered_ = false;
    registry_now_empty = sessions_by_slave_.empty();
  }

  // Only tear the dispatchers down once the last session is gone; doing it per
  // session would silence the hands that are still open.
  if (registry_now_empty)
  {
    set_can_tx_callback(&SocketCanSession::noop_tx_callback);
    set_can_rx_callback(&SocketCanSession::noop_rx_callback);
  }

  if (handle_)
  {
    handle_.reset();
  }
  clear_handler();

  close_socket();

  tx_fault_logged_ = false;
  rx_fault_logged_ = false;
  bus_error_logged_ = false;
}

bool SocketCanSession::is_open() const
{
  std::lock_guard<std::mutex> lock(socket_mutex_);
  return socket_fd_ != kInvalidSocketFd && static_cast<bool>(handle_);
}

std::optional<BraincoHandApi::ConnectionInfo> SocketCanSession::connection_info() const
{
  if (!is_open())
  {
    BRAINCO_HAND_LOG_WARN(
      "connection_info requested while SocketCAN session on '%s' is closed",
      config_.socketcan.interface.c_str());
    return std::nullopt;
  }

  BraincoHandApi::ConnectionInfo info{};
  info.port = config_.socketcan.interface;
  // Bit timing lives in the network interface, not in this session, so there is
  // no meaningful baudrate to report here — see `ip -d link show`.
  info.baudrate = 0;
  info.slave_id = config_.slave_id;
  return info;
}

bool SocketCanSession::open_socket()
{
  const auto & socketcan = config_.socketcan;

  if (socketcan.interface.empty())
  {
    BRAINCO_HAND_LOG_ERROR("SocketCAN interface name is empty; set the 'can_interface' parameter");
    return false;
  }
  if (socketcan.interface.size() >= IFNAMSIZ)
  {
    BRAINCO_HAND_LOG_ERROR(
      "SocketCAN interface name '%s' is %zu chars, limit is %d", socketcan.interface.c_str(),
      socketcan.interface.size(), IFNAMSIZ - 1);
    return false;
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0)
  {
    BRAINCO_HAND_LOG_ERROR("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: %s", std::strerror(errno));
    return false;
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, socketcan.interface.c_str(), IFNAMSIZ - 1);

  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
  {
    BRAINCO_HAND_LOG_ERROR(
      "interface '%s' not found: %s (is the CAN device up? try 'ip link show')",
      socketcan.interface.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }
  const int interface_index = ifr.ifr_ifindex;

  // Guard against the most common setup mistake: the interface exists but was
  // never brought up, which otherwise only shows as silent TX failures later.
  ifreq flags_request{};
  std::strncpy(flags_request.ifr_name, socketcan.interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFFLAGS, &flags_request) < 0)
  {
    BRAINCO_HAND_LOG_WARN(
      "could not read flags of '%s': %s; continuing without the link-up check",
      socketcan.interface.c_str(), std::strerror(errno));
  }
  else if ((flags_request.ifr_flags & IFF_UP) == 0)
  {
    BRAINCO_HAND_LOG_ERROR(
      "interface '%s' is DOWN; bring it up first, e.g. "
      "'ip link set %s up type can bitrate 1000000 dbitrate 5000000 fd on'",
      socketcan.interface.c_str(), socketcan.interface.c_str());
    ::close(fd);
    return false;
  }

  // CAN FD frames are mandatory for the Revo2 protocol: without this option the
  // kernel truncates every write() to a classic 8-byte frame.
  int enable_canfd = kSockOptEnable;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
  {
    BRAINCO_HAND_LOG_ERROR(
      "enabling CAN FD on '%s' failed: %s (interface is probably not CAN-FD capable, or was brought "
      "up without 'fd on')",
      socketcan.interface.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  // The SDK correlates replies with its own requests; echoed TX frames would be
  // parsed as bogus responses.
  int receive_own_messages = kSockOptDisable;
  if (
    ::setsockopt(
      fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own_messages,
      sizeof(receive_own_messages)) < 0)
  {
    BRAINCO_HAND_LOG_WARN(
      "disabling CAN_RAW_RECV_OWN_MSGS on '%s' failed: %s; own frames may be read back",
      socketcan.interface.c_str(), std::strerror(errno));
  }

  // A zero mask accepts every ID, which mirrors the vendor CAN FD session. Set a
  // non-zero mask when the hand shares the bus with other devices.
  if (socketcan.filter_can_mask != 0U)
  {
    can_filter filter{};
    filter.can_id = socketcan.filter_can_id;
    filter.can_mask = socketcan.filter_can_mask;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
    {
      BRAINCO_HAND_LOG_ERROR(
        "applying CAN filter (id=0x%X mask=0x%X) on '%s' failed: %s", socketcan.filter_can_id,
        socketcan.filter_can_mask, socketcan.interface.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }
    BRAINCO_HAND_LOG_INFO(
      "CAN filter active on '%s': id=0x%X mask=0x%X", socketcan.interface.c_str(),
      socketcan.filter_can_id, socketcan.filter_can_mask);
  }
  else
  {
    BRAINCO_HAND_LOG_INFO(
      "no CAN filter on '%s'; every frame on the bus is accepted", socketcan.interface.c_str());
  }

  const timeval send_timeout = to_timeval(kSendTimeoutMs);
  if (::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout)) < 0)
  {
    BRAINCO_HAND_LOG_WARN(
      "setting the %ld ms send timeout on '%s' failed: %s; a full TX queue may block",
      static_cast<long>(kSendTimeoutMs), socketcan.interface.c_str(), std::strerror(errno));
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_index;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
  {
    BRAINCO_HAND_LOG_ERROR(
      "bind to '%s' failed: %s", socketcan.interface.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  std::lock_guard<std::mutex> lock(socket_mutex_);
  socket_fd_ = fd;
  return true;
}

void SocketCanSession::close_socket()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ == kInvalidSocketFd)
  {
    return;
  }

  if (::close(socket_fd_) < 0)
  {
    BRAINCO_HAND_LOG_WARN(
      "closing the socket for '%s' failed: %s", config_.socketcan.interface.c_str(),
      std::strerror(errno));
  }
  socket_fd_ = kInvalidSocketFd;
}

int32_t SocketCanSession::tx_callback(
  uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
{
  auto * session = lookup_session(slave_id);
  if (session == nullptr)
  {
    BRAINCO_HAND_LOG_ERROR_THROTTLE(
      kDispatchLogThrottleMs, "TX for slave %u has no open session (throttled to %lldms)",
      static_cast<unsigned>(slave_id), static_cast<long long>(kDispatchLogThrottleMs));
    return -1;
  }
  return session->handle_tx(slave_id, can_id, data, data_len);
}

int32_t SocketCanSession::rx_callback(
  uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
{
  auto * session = lookup_session(slave_id);
  if (session == nullptr)
  {
    BRAINCO_HAND_LOG_ERROR_THROTTLE(
      kDispatchLogThrottleMs, "RX for slave %u has no open session (throttled to %lldms)",
      static_cast<unsigned>(slave_id), static_cast<long long>(kDispatchLogThrottleMs));
    return -1;
  }
  return session->handle_rx(slave_id, can_id_out, data_out, data_len_out);
}

int32_t SocketCanSession::noop_tx_callback(uint8_t, uint32_t, const uint8_t *, uintptr_t)
{
  return -1;
}

int32_t SocketCanSession::noop_rx_callback(uint8_t, uint32_t *, uint8_t *, uintptr_t *)
{
  return -1;
}

int32_t SocketCanSession::handle_tx(
  uint8_t slave_id, uint32_t can_id, const uint8_t * data, uintptr_t data_len)
{
  (void)slave_id;

  if (data == nullptr && data_len > 0)
  {
    BRAINCO_HAND_LOG_ERROR("TX rejected: null payload with data_len=%zu", data_len);
    return -1;
  }
  if (data_len > CANFD_MAX_DLEN)
  {
    BRAINCO_HAND_LOG_WARN(
      "TX payload of %zu bytes exceeds the CAN FD limit of %d, truncating", data_len,
      CANFD_MAX_DLEN);
  }

  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ == kInvalidSocketFd)
  {
    if (!tx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR("TX attempted while the SocketCAN session is closed");
      tx_fault_logged_ = true;
    }
    return -1;
  }

  canfd_frame frame{};
  frame.can_id = config_.socketcan.use_extended_id ? ((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                                                   : (can_id & CAN_SFF_MASK);
  frame.len =
    static_cast<uint8_t>(std::min<uintptr_t>(data_len, static_cast<uintptr_t>(CANFD_MAX_DLEN)));
  frame.flags = config_.socketcan.enable_bitrate_switch ? CANFD_BRS : 0;
  if (frame.len > 0)
  {
    std::memcpy(frame.data, data, frame.len);
  }

  // A full netdev TX queue surfaces as ENOBUFS. That is back-pressure, not a
  // fault: on a bus shared with an arm the arm always wins arbitration (ids
  // 0x01-0x18 beat this hand's base id 0x1F), so our frames wait for a gap and
  // the queue -- ten slots by kernel default -- briefly fills.
  //
  // It cannot be waited out through the socket API. ENOBUFS comes back
  // synchronously from dev_queue_xmit; SO_SNDTIMEO governs only send-buffer
  // allocation, and POLLOUT on a raw CAN socket reports socket buffer space
  // rather than qdisc occupancy, so it signals ready immediately. A bounded
  // retry is the only thing that actually helps.
  ssize_t written = -1;
  for (int attempt = 0; attempt <= kTxEnobufsRetries; ++attempt)
  {
    written = ::write(socket_fd_, &frame, static_cast<size_t>(CANFD_MTU));
    if (written == static_cast<ssize_t>(CANFD_MTU) || errno != ENOBUFS)
    {
      break;
    }
    ++tx_backpressure_events_;
    std::this_thread::sleep_for(kTxEnobufsBackoff);
  }

  if (written != static_cast<ssize_t>(CANFD_MTU))
  {
    if (!tx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR(
        "TX failed on '%s' (can_id=0x%X len=%u): wrote %zd of %d bytes: %s",
        config_.socketcan.interface.c_str(), can_id, static_cast<unsigned>(frame.len), written,
        CANFD_MTU, std::strerror(errno));
      tx_fault_logged_ = true;
    }
    return -1;
  }

  // Retries that eventually succeeded still say the bus is near capacity, so
  // report the running count on a throttle rather than staying silent.
  if (tx_backpressure_events_ > 0)
  {
    BRAINCO_HAND_LOG_INFO_THROTTLE(
      kBackpressureSummaryThrottleMs,
      "'%s': TX queue was full %zu time(s) so far, each cleared by retry (throttled to %lldms)",
      config_.socketcan.interface.c_str(), tx_backpressure_events_,
      static_cast<long long>(kBackpressureSummaryThrottleMs));
  }

  if (tx_fault_logged_)
  {
    BRAINCO_HAND_LOG_INFO("TX recovered on '%s'", config_.socketcan.interface.c_str());
    tx_fault_logged_ = false;
  }
  return 0;
}

int32_t SocketCanSession::handle_rx(
  uint8_t slave_id, uint32_t * can_id_out, uint8_t * data_out, uintptr_t * data_len_out)
{
  (void)slave_id;

  if (can_id_out == nullptr || data_out == nullptr || data_len_out == nullptr)
  {
    BRAINCO_HAND_LOG_ERROR(
      "RX rejected: null output pointer (can_id=%p data=%p len=%p)",
      static_cast<const void *>(can_id_out), static_cast<const void *>(data_out),
      static_cast<const void *>(data_len_out));
    return -1;
  }

  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ == kInvalidSocketFd)
  {
    if (!rx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR("RX attempted while the SocketCAN session is closed");
      rx_fault_logged_ = true;
    }
    return -1;
  }

  pollfd poll_request{};
  poll_request.fd = socket_fd_;
  poll_request.events = POLLIN;

  const int ready =
    ::poll(&poll_request, 1, static_cast<int>(config_.socketcan.rx_wait_time_ms));
  if (ready == kPollTimeout)
  {
    // No reply within the window. This is an ordinary outcome — the SDK retries —
    // so it must not be logged on the control path.
    return -1;
  }
  if (ready < 0)
  {
    if (errno != EINTR && !rx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR(
        "poll on '%s' failed: %s", config_.socketcan.interface.c_str(), std::strerror(errno));
      rx_fault_logged_ = true;
    }
    return -1;
  }

  canfd_frame frame{};
  const ssize_t received = ::read(socket_fd_, &frame, sizeof(frame));
  if (received < 0)
  {
    if (errno != EINTR && errno != EAGAIN && !rx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR(
        "read on '%s' failed: %s", config_.socketcan.interface.c_str(), std::strerror(errno));
      rx_fault_logged_ = true;
    }
    return -1;
  }

  // A CAN FD socket also delivers classic frames, which are CAN_MTU long. Both
  // layouts share the can_id/len/data offsets, so either size is usable as-is.
  if (received != static_cast<ssize_t>(CANFD_MTU) && received != static_cast<ssize_t>(CAN_MTU))
  {
    if (!rx_fault_logged_)
    {
      BRAINCO_HAND_LOG_ERROR(
        "RX on '%s' returned %zd bytes, expected %d or %d", config_.socketcan.interface.c_str(),
        received, CANFD_MTU, CAN_MTU);
      rx_fault_logged_ = true;
    }
    return -1;
  }

  // Error frames carry bus diagnostics, never payload; surfacing them here is
  // what makes bus-off / ACK failures visible without an external analyzer.
  if ((frame.can_id & CAN_ERR_FLAG) != 0U)
  {
    if (!bus_error_logged_)
    {
      BRAINCO_HAND_LOG_ERROR(
        "CAN error frame on '%s' (class=0x%X); check termination, bitrate and wiring — "
        "'ip -d -s link show %s' has the counters",
        config_.socketcan.interface.c_str(), frame.can_id & CAN_ERR_MASK,
        config_.socketcan.interface.c_str());
      bus_error_logged_ = true;
    }
    return -1;
  }

  const auto payload_len =
    static_cast<uint8_t>(std::min<uintptr_t>(frame.len, static_cast<uintptr_t>(CANFD_MAX_DLEN)));

  // The SDK expects a bare identifier: strip the EFF/RTR/ERR flag bits.
  *can_id_out =
    (frame.can_id & CAN_EFF_FLAG) != 0U ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
  *data_len_out = payload_len;
  if (payload_len > 0)
  {
    std::memcpy(data_out, frame.data, payload_len);
  }

  if (rx_fault_logged_ || bus_error_logged_)
  {
    BRAINCO_HAND_LOG_INFO("RX recovered on '%s'", config_.socketcan.interface.c_str());
    rx_fault_logged_ = false;
    bus_error_logged_ = false;
  }
  return 0;
}

}  // namespace brainco_hand_driver
