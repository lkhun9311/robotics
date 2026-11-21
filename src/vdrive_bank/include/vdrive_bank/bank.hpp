// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "drive_core/drive.hpp"
#include "runtime_core/time.hpp"
#include "vdrive_bank/entropy.hpp"

// Six virtual drives (node IDs 1..6) behind one CAN interface, in logic time.
// The bank routes received frames by COB-ID (B2), produces each node's heartbeat (B3) and EMCY
// (B4), and applies injected faults (B6).
// It does no I/O: the caller passes every frame and a CLOCK_MONOTONIC instant, sends what comes
// back, and receives fixed-size records through the sink.
namespace vdrive_bank {

// 0x1017 producer heartbeat time.
inline constexpr runtime_core::Duration kHeartbeatPeriod{
    static_cast<std::int64_t>(drive_core::kProducerHeartbeatMs) * 1'000'000};

// One traced fact.
// Only the fields named next to the kind are meaningful; `node` is 0 for a record that belongs to
// no node.
struct BankRecord {
  enum class Kind : std::uint8_t {
    kDrive,           // drive: an event of the node's Drive.
    kNmt,             // nmt_state: an NMT command reached the node; its state afterwards.
    kHeartbeat,       // nmt_state: a heartbeat sent (0x00 is the boot-up message).
    kEmcySent,        // emcy_code: an EMCY sent (0x0000 is the error reset).
    kEmcySuppressed,  // emcy_code: an EMCY not sent because of the NMT state (B4).
    kIgnored,         // cob_id: a received frame no node consumes (B2).
    kInject,          // text: "ok <command>" or "err <reason>: <command>", truncated.
    kOverload,        // Reserved for the hosting process (control loop overrun).
  };
  Kind kind{};
  std::uint8_t node{};
  runtime_core::MonoTime at{};
  drive_core::DriveEvent drive{};
  std::uint8_t nmt_state{};
  std::uint16_t emcy_code{};
  std::uint16_t cob_id{};
  char text[48]{};  // NUL-terminated.
};
static_assert(std::is_trivially_copyable_v<BankRecord>);

using RecordSink = std::function<void(const BankRecord&)>;

// Received frames that reached no node, by CiA 301 COB-ID class (B2).
struct IgnoredCounts {
  std::uint32_t nmt_unknown_node{};  // NMT addressed to a node outside 1..6.
  std::uint32_t emcy{};              // 0x081..0x0FF: another producer's EMCY.
  std::uint32_t pdo{};               // 0x181..0x57F not addressed to a node's RPDO1-3.
  std::uint32_t sdo{};               // 0x581..0x5FF, and requests to a node outside 1..6.
  std::uint32_t heartbeat{};         // 0x701..0x77F: another node's heartbeat.
  std::uint32_t other{};             // Anything else (TIME, LSS, unassigned identifiers).

  friend bool operator==(const IgnoredCounts&, const IgnoredCounts&) = default;
};

class Bank {
 public:
  struct InjectResult {
    bool ok{};
    std::string error;  // Empty when ok.
    // Frames the injection produces (an EMCY), to be sent like the frames of any other call.
    std::vector<canopen_codec::Frame> frames;
  };

  // boot_ids: the first boot_id of nodes 1..6 (draw_boot_ids).
  // Throws std::invalid_argument unless they are nonzero and distinct.
  Bank(std::array<std::uint32_t, kNodeCount> boot_ids, runtime_core::MonoTime now);
  // Each Drive and its event sink refer to the bank, so it stays where it was made.
  Bank(const Bank&) = delete;
  Bank& operator=(const Bank&) = delete;
  ~Bank();

  // Every call below throws std::logic_error if `now` is earlier than the previous call's (or
  // the constructor's): time passed to a Drive never decreases.

  // Powers on all six nodes in node order: returns the six boot-up messages, and the heartbeat
  // phase of each node starts here.
  std::vector<canopen_codec::Frame> power_on(runtime_core::MonoTime now);
  // The W tick of all six drives in node order, then the periodic heartbeats that are due.
  // Nothing runs before power_on.
  std::vector<canopen_codec::Frame> tick(runtime_core::MonoTime now);
  // Routes one received frame (B2) and returns the frames to send: replies of the drives, a
  // boot-up after an NMT reset, and EMCYs caused by the frame (an error reset after Fault Reset).
  std::vector<canopen_codec::Frame> on_frame(const canopen_codec::Frame& frame,
                                             runtime_core::MonoTime now);
  // B6: `fault <node> <hex>`, `torque_loss <node>`, `stuck <node> 0|1`, `hb_mute <node> 0|1`
  // with node 1..6 and hex 1..4 hex digits.
  // A rejected command changes nothing.
  InjectResult inject(std::string_view command, runtime_core::MonoTime now);

  void set_sink(RecordSink sink) { sink_ = std::move(sink); }

  // node 1..6; throws std::out_of_range otherwise.
  const drive_core::Drive& drive(std::uint8_t node) const;
  bool heartbeat_muted(std::uint8_t node) const;
  const IgnoredCounts& ignored() const { return ignored_; }

 private:
  struct Node;

  void check_time(runtime_core::MonoTime now);
  Node& node_at(std::uint8_t node) const;
  // Runs one call into a drive and turns what it did into frames and records: the returned
  // frames (a boot-up restarts the heartbeat phase), its events, an EMCY per accepted fault and
  // an error reset when the fault is gone.
  template <typename Call>
  void run(Node& n, runtime_core::MonoTime now, std::vector<canopen_codec::Frame>& out,
           Call&& call);
  void send_emcy(Node& n, std::uint16_t code, runtime_core::MonoTime now,
                 std::vector<canopen_codec::Frame>& out);
  void ignore(const canopen_codec::Frame& frame, runtime_core::MonoTime now);
  void record(BankRecord r) const;
  void record_inject(std::string_view text, runtime_core::MonoTime now) const;

  std::array<std::unique_ptr<Node>, kNodeCount> nodes_;
  runtime_core::MonoTime last_;
  bool powered_{false};
  IgnoredCounts ignored_;
  RecordSink sink_;
};

}  // namespace vdrive_bank
