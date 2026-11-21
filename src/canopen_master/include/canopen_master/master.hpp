// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "canopen_codec/codec.hpp"
#include "canopen_master/events.hpp"
#include "canopen_master/heartbeat.hpp"
#include "canopen_master/sdo_client.hpp"
#include "runtime_core/time.hpp"

// The CANopen master core (C5): NMT master, heartbeat consumer (C7), expedited SDO client (C8),
// SYNC producer, EMCY consumer and received-PDO decoding through registered layouts.
// Pure logic in the shape of drive_core::Drive: frames in through on_frame, time in through tick,
// frames to send out as return values, events drained by the caller.
// No sockets, no threads, no clock reads: every time comes from the caller.
//
// Commands (nmt, sdo_upload, sdo_download, set_heartbeat_consumer) are queued and take effect at
// the next on_frame or tick, whose `now` they use and whose returned frames carry them.
//
// on_frame expires what is due at `now` (equality is expired), handles its frame against the
// state from before any queued command, runs the SDO callbacks, then applies the queued commands:
// a frame received before the call never answers a request or reset that the call sends.
// tick applies the queued commands, expires, runs the callbacks, applies what they queued, then
// produces a SYNC when one is due.
// If an SDO callback throws, the exception leaves the call and the callbacks not yet called run at
// the next call.
//
// The master sends an NMT command only when asked through nmt(): a lost heartbeat or a boot
// timeout is an event, never a reset or a stop (roadmap boot policy).
//
// Not thread-safe.
// The caller serializes every call (in E3c the I/O worker owns the master).
namespace canopen_master {

inline constexpr runtime_core::Duration kDefaultSyncPeriod{10'000'000};         // T_bus
inline constexpr runtime_core::Duration kDefaultHeartbeatTimeout{250'000'000};  // C7
inline constexpr runtime_core::Duration kDefaultBootTimeout{1'000'000'000};     // C7
inline constexpr runtime_core::Duration kDefaultSdoTimeout{200'000'000};        // C8

struct MasterConfig {
  std::vector<canopen_codec::NodeId> nodes;
  // SYNC slots lie on a grid of this period from the first SYNC; zero or less produces none.
  runtime_core::Duration sync_period{kDefaultSyncPeriod};
  // Consumer time of every node until set_heartbeat_consumer changes it; zero disables.
  runtime_core::Duration heartbeat_timeout{kDefaultHeartbeatTimeout};
  // First-heartbeat window, from the consumer's start or the master's last NMT reset of the node.
  runtime_core::Duration boot_timeout{kDefaultBootTimeout};
  runtime_core::Duration sdo_timeout{kDefaultSdoTimeout};
};

// Frames the master received and did not act on.
struct MasterCounts {
  std::uint32_t unregistered_cob{};     // A COB-ID the master does not consume.
  std::uint32_t foreign{};              // A configured service of a node not in the config, or
                                        // another master's NMT.
  std::uint32_t malformed{};            // A heartbeat or EMCY that did not decode (DLC, state).
  std::uint32_t sdo_unexpected{};       // An SDO response with no request outstanding.
  std::uint32_t sdo_protocol_errors{};  // Requests failed as kSdoProtocol.
  std::uint32_t sdo_refused{};          // Requests refused when queued.
  std::uint32_t pdo_length_errors{};    // A registered PDO whose DLC is not its layout's length.

  friend bool operator==(const MasterCounts&, const MasterCounts&) = default;
};

// One mapped object of a received PDO, in mapping order from bit 0 of byte 0 (CiA 301).
struct PdoField {
  std::uint16_t index{};
  std::uint8_t sub{};
  std::uint8_t bit_length{};  // 1..64
  bool is_signed{};
};

// The vdrive layouts come from the generated EDS header; a CANopenNode layout is the map the
// master wrote to it.
struct PdoLayout {
  std::vector<PdoField> fields;
};

enum class SyncControl : std::uint8_t {
  kPause,   // No SYNC until kStep or kResume.
  kResume,  // Back on the period: the next SYNC one period after the last, then a new grid.
  kStep,    // While paused: one SYNC at the next tick, at least half a period after the last.
};

class Master {
 public:
  // Throws std::invalid_argument for an empty or duplicate node list or a non-positive boot or
  // SDO timeout.
  explicit Master(MasterConfig config);

  // One received frame at `now`; returns the frames to send.
  std::vector<canopen_codec::Frame> on_frame(const canopen_codec::Frame& frame,
                                             runtime_core::MonoTime now);
  // Time passes to `now`; returns the frames to send.
  std::vector<canopen_codec::Frame> tick(runtime_core::MonoTime now);

  // Returns the events since the last call, oldest first, and clears them.
  std::vector<MasterEvent> drain_events();

  // Queue an NMT command for a configured node. false for a node not in the config.
  // A Reset Node or Reset Communication restarts that node's boot window when it is sent.
  bool nmt(canopen_codec::NmtCommand command, canopen_codec::NodeId node);

  // Queue an expedited SDO request. false (counted in sdo_refused; `done` never called) for a
  // node not in the config, a full queue (SdoClient::kCapacity per node) or a download of other
  // than 1..4 bytes.
  // `done` may be empty.
  // `tag` (nonzero) lets cancel_sdo drop the request while it is still queued.
  bool sdo_upload(canopen_codec::NodeId node, std::uint16_t index, std::uint8_t sub, SdoDone done,
                  std::uint64_t tag = 0);
  bool sdo_download(canopen_codec::NodeId node, std::uint16_t index, std::uint8_t sub,
                    std::span<const std::uint8_t> bytes, SdoDone done, std::uint64_t tag = 0);
  // Drops every request with this nonzero tag that has not been sent yet, on every node; their
  // `done` is never called.
  // A request already sent is not recalled.
  // Returns how many dropped.
  std::size_t cancel_sdo(std::uint64_t tag);

  // Per-node consumer time; zero disables that node's consumer (C7). false for a node not in
  // the config.
  bool set_heartbeat_consumer(canopen_codec::NodeId node, runtime_core::Duration consumer_time);

  // Decode PDOs received on `cob` (an 11-bit COB-ID) as `layout` for `node`; registering a
  // COB-ID again replaces its layout.
  // A frame whose DLC is not the layout's byte length is a kPdoLengthError and is not decoded.
  // false for a node not in the config, a COB-ID of NMT, SYNC, EMCY, SDO or heartbeat, or a layout
  // that is empty, has a zero-length field or holds more than 64 bits.
  bool register_rx_pdo(canopen_codec::NodeId node, std::uint32_t cob, PdoLayout layout);

  // SYNC production (for the N5 interop row): pause, resume or step once. kStep returns false
  // unless paused, while a step is not yet produced, or with sync_period zero; a pause drops a
  // step not yet produced.
  bool sync_control(SyncControl control);
  std::uint64_t syncs_produced() const { return syncs_; }

  // The NMT state from the node's last heartbeat; std::nullopt before any, or after the master
  // reset the node. kBootUp after a boot-up message.
  std::optional<canopen_codec::NmtState> nmt_state(canopen_codec::NodeId node) const;
  std::optional<HeartbeatStatus> heartbeat_status(canopen_codec::NodeId node) const;
  const MasterCounts& counts() const { return counts_; }
  // The node's last EMCY (every one is also a kEmcy event).
  std::optional<EmcyDetail> last_emcy(canopen_codec::NodeId node) const;

 private:
  struct Node {
    canopen_codec::NodeId id;
    HeartbeatConsumer heartbeat;
    SdoClient sdo;
    std::optional<canopen_codec::NmtState> state;
    std::optional<EmcyDetail> last_emcy;
  };
  struct RxPdo {
    std::uint16_t cob;
    std::uint8_t node;
    std::uint8_t bytes;
    PdoLayout layout;
  };
  struct Done {
    SdoDone done;
    SdoResult result;
  };

  Node* find(canopen_codec::NodeId node);
  const Node* find(canopen_codec::NodeId node) const;
  Node* find(std::uint16_t cob, std::uint16_t base);
  void emit(MasterEventKind kind, std::uint8_t node, EventDetail detail = {});
  void apply_commands(runtime_core::MonoTime now, std::vector<canopen_codec::Frame>& out);
  void expire(runtime_core::MonoTime now, std::vector<canopen_codec::Frame>& out);
  void handle(const canopen_codec::Frame& frame, std::vector<canopen_codec::Frame>& out);
  void handle_heartbeat(Node& node, const canopen_codec::Frame& frame);
  void handle_emcy(Node& node, const canopen_codec::Frame& frame);
  void handle_pdo(const RxPdo& pdo, const canopen_codec::Frame& frame);
  void produce_sync(runtime_core::MonoTime now, std::vector<canopen_codec::Frame>& out);
  void complete(SdoCompletion completion, std::vector<canopen_codec::Frame>& out);
  void run_callbacks();
  // The common body of on_frame and tick.
  std::vector<canopen_codec::Frame> step(const canopen_codec::Frame* frame,
                                         runtime_core::MonoTime now, bool produce);

  MasterConfig config_;
  std::vector<Node> nodes_;
  std::array<std::int16_t, 128> slot_{};  // Node-ID -> index in nodes_, -1 when not configured.
  bool started_{};
  bool in_step_{};
  runtime_core::MonoTime now_{};
  std::vector<canopen_codec::NmtMessage> pending_nmt_;
  std::vector<std::pair<canopen_codec::NodeId, runtime_core::Duration>> pending_consumer_;
  std::vector<Done> pending_done_;
  std::vector<MasterEvent> events_;
  MasterCounts counts_;
  std::vector<RxPdo> rx_pdos_;
  bool sync_paused_{};
  bool sync_step_{};
  bool sync_reanchor_{true};  // The next SYNC starts a new grid (first SYNC, or after a resume).
  runtime_core::MonoTime sync_due_{};
  std::optional<runtime_core::MonoTime> last_sync_;
  std::uint64_t syncs_{};
};

}  // namespace canopen_master
