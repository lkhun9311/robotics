// SPDX-License-Identifier: Apache-2.0
#include "vdrive_bank/bank.hpp"

#include <algorithm>
#include <charconv>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <utility>

namespace vdrive_bank {

namespace cc = canopen_codec;
using drive_core::Drive;
using drive_core::DriveEvent;
using drive_core::DriveEventKind;
using runtime_core::Duration;
using runtime_core::MonoTime;

struct Bank::Node {
  std::uint8_t id{};
  std::unique_ptr<Drive> drive;
  MonoTime next_heartbeat{};  // Phase of the periodic heartbeat: the latest boot-up + k * period.
  bool muted{false};
  std::vector<DriveEvent> events;  // Collected by the drive's sink during one call.
};

namespace {

// CiA 301 predefined connection set: the receive COB-ID bases the bank routes to node n.
constexpr std::uint16_t kNmtCobId = 0x000;
constexpr std::array<std::uint16_t, 4> kNodeRxBases{0x200, 0x300, 0x400, 0x600};  // RPDO1-3, SDO

bool is_boot_up(const cc::Frame& f, std::uint8_t node) {
  return f.cob_id == 0x700U + node && f.dlc == 1 && f.data[0] == 0x00;
}

// CiA 301: EMCY is produced in Pre-operational and Operational only (D12 matrix).
bool emcy_allowed(cc::NmtState state) {
  return state == cc::NmtState::kPreOperational || state == cc::NmtState::kOperational;
}

// 1001h as drive_core serves it by SDO: bit 0 (generic error) while a fault is being handled.
std::uint8_t error_register(const Drive& d) { return d.cia402().fault_code() ? 0x01U : 0x00U; }

BankRecord make_record(BankRecord::Kind kind, std::uint8_t node, MonoTime at) {
  BankRecord r{};
  r.kind = kind;
  r.node = node;
  r.at = at;
  return r;
}

// The node 1..kNodeCount of a one-character decimal token.
std::optional<std::uint8_t> parse_node(std::string_view s) {
  if (s.size() != 1 || s[0] < '1' || s[0] > static_cast<char>('0' + kNodeCount)) {
    return std::nullopt;
  }
  return static_cast<std::uint8_t>(s[0] - '0');
}

// 1..4 hex digits, no prefix or sign.
std::optional<std::uint16_t> parse_hex16(std::string_view s) {
  if (s.empty() || s.size() > 4 || !std::all_of(s.begin(), s.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
      })) {
    return std::nullopt;
  }
  std::uint16_t v = 0;
  std::from_chars(s.data(), s.data() + s.size(), v, 16);
  return v;
}

std::optional<bool> parse_flag(std::string_view s) {
  if (s == "0") {
    return false;
  }
  if (s == "1") {
    return true;
  }
  return std::nullopt;
}

// Space-separated tokens; an empty token (leading, trailing or double space) is kept, so the
// command is rejected rather than read loosely.
std::vector<std::string_view> split(std::string_view s) {
  std::vector<std::string_view> out;
  if (s.empty()) {
    return out;
  }
  std::size_t start = 0;
  while (true) {
    const std::size_t space = s.find(' ', start);
    out.push_back(s.substr(start, space - start));
    if (space == std::string_view::npos) {
      return out;
    }
    start = space + 1;
  }
}

// One B6 command, parsed in full before anything is applied.
struct Injection {
  enum class Verb : std::uint8_t { kFault, kTorqueLoss, kStuck, kHeartbeatMute };
  Verb verb{};
  std::uint8_t node{};
  std::uint16_t code{};  // kFault
  bool flag{};           // kStuck, kHeartbeatMute
};

// nullptr when `command` parsed into `in`, else why it was rejected.
const char* parse_injection(std::string_view command, Injection& in) {
  const std::vector<std::string_view> tok = split(command);
  if (tok.empty()) {
    return "empty command";
  }
  std::size_t arity = 3;
  if (tok[0] == "fault") {
    in.verb = Injection::Verb::kFault;
  } else if (tok[0] == "torque_loss") {
    in.verb = Injection::Verb::kTorqueLoss;
    arity = 2;
  } else if (tok[0] == "stuck") {
    in.verb = Injection::Verb::kStuck;
  } else if (tok[0] == "hb_mute") {
    in.verb = Injection::Verb::kHeartbeatMute;
  } else {
    return "unknown command";
  }
  if (tok.size() != arity) {
    return "wrong argument count";
  }
  if (const std::optional<std::uint8_t> node = parse_node(tok[1])) {
    in.node = *node;
  } else {
    return "bad node";
  }
  if (in.verb == Injection::Verb::kFault) {
    // CiA 301: EMCY code 0x0000 is the error reset, so it can never name a fault.
    if (const std::optional<std::uint16_t> code = parse_hex16(tok[2]); code && *code != 0U) {
      in.code = *code;
    } else {
      return "bad error code";
    }
  } else if (arity == 3) {
    if (const std::optional<bool> flag = parse_flag(tok[2])) {
      in.flag = *flag;
    } else {
      return "bad flag";
    }
  }
  return nullptr;
}

}  // namespace

Bank::Bank(std::array<std::uint32_t, kNodeCount> boot_ids, MonoTime now) : last_(now) {
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    if (boot_ids[i] == 0U ||
        std::find(boot_ids.begin(), boot_ids.begin() + static_cast<std::ptrdiff_t>(i),
                  boot_ids[i]) != boot_ids.begin() + static_cast<std::ptrdiff_t>(i)) {
      throw std::invalid_argument("vdrive_bank: boot_ids must be nonzero and distinct");
    }
  }
  for (std::size_t i = 0; i < kNodeCount; ++i) {
    auto node = std::make_unique<Node>();
    node->id = static_cast<std::uint8_t>(i + 1);
    const std::optional<cc::NodeId> id = cc::NodeId::make(node->id);
    if (!id) {
      throw std::logic_error("vdrive_bank: node ID out of range");  // 1..6 always is in range.
    }
    node->drive = std::make_unique<Drive>(*id, boot_ids[i], now);
    node->events.reserve(16);
    Node* raw = node.get();
    node->drive->set_event_sink([raw](const DriveEvent& e) { raw->events.push_back(e); });
    nodes_[i] = std::move(node);
  }
}

Bank::~Bank() = default;

void Bank::check_time(MonoTime now) {
  if (now < last_) {
    throw std::logic_error("vdrive_bank: time passed to the bank went backwards");
  }
  last_ = now;
}

Bank::Node& Bank::node_at(std::uint8_t node) const {
  if (node < 1 || node > kNodeCount) {
    throw std::out_of_range("vdrive_bank: node outside 1..6");
  }
  return *nodes_[node - 1U];
}

const Drive& Bank::drive(std::uint8_t node) const { return *node_at(node).drive; }

bool Bank::heartbeat_muted(std::uint8_t node) const { return node_at(node).muted; }

void Bank::record(BankRecord r) const {
  if (sink_) {
    sink_(r);
  }
}

void Bank::record_inject(std::string_view text, MonoTime now) const {
  BankRecord r = make_record(BankRecord::Kind::kInject, 0, now);
  const std::size_t n = std::min(text.size(), sizeof(r.text) - 1);
  std::memcpy(r.text, text.data(), n);
  r.text[n] = '\0';
  record(r);
}

template <typename Call>
void Bank::run(Node& n, MonoTime now, std::vector<cc::Frame>& out, Call&& call) {
  const bool had_fault = n.drive->cia402().fault_code().has_value();
  const std::vector<cc::Frame> frames = std::forward<Call>(call)(*n.drive);
  for (const cc::Frame& f : frames) {
    if (is_boot_up(f, n.id)) {
      // B3: power-on, reset node and reset communication; the periodic phase restarts here.
      n.next_heartbeat = now + kHeartbeatPeriod;
      BankRecord r = make_record(BankRecord::Kind::kHeartbeat, n.id, now);
      r.nmt_state = 0x00;
      r.cob_id = f.cob_id;
      record(r);
    }
    out.push_back(f);
  }
  for (const DriveEvent& e : n.events) {
    BankRecord r = make_record(BankRecord::Kind::kDrive, n.id, now);
    r.drive = e;
    record(r);
    if (e.kind == DriveEventKind::kFault) {
      // drive_core emits kFault only for an accepted fault (transition 13 or torque loss).
      send_emcy(n, e.fault_code, now, out);
    }
  }
  n.events.clear();
  if (had_fault && !n.drive->cia402().fault_code()) {
    send_emcy(n, 0x0000, now, out);  // Error reset: the drive left Fault.
  }
}

void Bank::send_emcy(Node& n, std::uint16_t code, MonoTime now, std::vector<cc::Frame>& out) {
  const cc::NmtState state = n.drive->nmt().state();
  const bool allowed = emcy_allowed(state);
  BankRecord r = make_record(
      allowed ? BankRecord::Kind::kEmcySent : BankRecord::Kind::kEmcySuppressed, n.id, now);
  r.emcy_code = code;
  r.nmt_state = static_cast<std::uint8_t>(state);
  if (allowed) {
    const cc::Frame f = cc::encode_emcy({n.drive->node(), code, error_register(*n.drive), {}});
    r.cob_id = f.cob_id;
    out.push_back(f);
  }
  record(r);
}

void Bank::ignore(const cc::Frame& frame, MonoTime now) {
  const std::uint16_t cob = frame.cob_id;
  if (cob == kNmtCobId) {
    ++ignored_.nmt_unknown_node;
  } else if (cob > 0x080 && cob <= 0x0FF) {
    ++ignored_.emcy;
  } else if (cob > 0x180 && cob <= 0x57F) {
    ++ignored_.pdo;
  } else if ((cob > 0x580 && cob <= 0x5FF) || (cob > 0x600 && cob <= 0x67F)) {
    ++ignored_.sdo;
  } else if (cob > 0x700 && cob <= 0x77F) {
    ++ignored_.heartbeat;
  } else {
    ++ignored_.other;
  }
  BankRecord r = make_record(BankRecord::Kind::kIgnored, 0, now);
  r.cob_id = cob;
  record(r);
}

std::vector<cc::Frame> Bank::power_on(MonoTime now) {
  check_time(now);
  std::vector<cc::Frame> out;
  for (const auto& n : nodes_) {
    run(*n, now, out, [now](Drive& d) { return d.power_on(now); });
  }
  powered_ = true;
  return out;
}

std::vector<cc::Frame> Bank::tick(MonoTime now) {
  check_time(now);
  std::vector<cc::Frame> out;
  if (!powered_) {
    return out;
  }
  for (const auto& n : nodes_) {
    run(*n, now, out, [now](Drive& d) {
      d.tick(now);
      return std::vector<cc::Frame>{};
    });
  }
  for (const auto& n : nodes_) {
    if (now < n->next_heartbeat) {
      continue;
    }
    // One heartbeat however late the tick; the next stays on the phase of the latest boot-up.
    const std::int64_t periods = (now - n->next_heartbeat).ns / kHeartbeatPeriod.ns + 1;
    n->next_heartbeat = n->next_heartbeat + Duration{periods * kHeartbeatPeriod.ns};
    if (n->muted) {
      continue;  // B6 hb_mute: periodic heartbeats only; boot-up is never muted.
    }
    const cc::NmtState state = n->drive->nmt().state();
    const cc::Frame f = cc::encode_heartbeat({n->drive->node(), state});
    out.push_back(f);
    BankRecord r = make_record(BankRecord::Kind::kHeartbeat, n->id, now);
    r.nmt_state = static_cast<std::uint8_t>(state);
    r.cob_id = f.cob_id;
    record(r);
  }
  return out;
}

std::vector<cc::Frame> Bank::on_frame(const cc::Frame& frame, MonoTime now) {
  check_time(now);
  std::vector<cc::Frame> out;
  const auto deliver = [&](Node& n) {
    run(n, now, out, [&frame, now](Drive& d) { return d.on_frame(frame, now); });
  };
  const std::uint16_t cob = frame.cob_id;
  if (cob == kNmtCobId) {
    const std::optional<cc::NmtMessage> m = cc::decode_nmt(frame);
    if (m && m->target && m->target->value() > kNodeCount) {
      ignore(frame, now);
      return out;
    }
    // Node byte 0 reaches all six; a malformed NMT goes to all six too, which count it.
    for (const auto& n : nodes_) {
      if (m && m->target && m->target->value() != n->id) {
        continue;
      }
      deliver(*n);
      if (m) {
        BankRecord r = make_record(BankRecord::Kind::kNmt, n->id, now);
        r.nmt_state = static_cast<std::uint8_t>(n->drive->nmt().state());
        record(r);
      }
    }
    return out;
  }
  if (cob == canopen_codec::od::kSyncCobId) {
    for (const auto& n : nodes_) {
      deliver(*n);
    }
    return out;
  }
  for (const std::uint16_t base : kNodeRxBases) {
    if (cob > base && cob <= base + kNodeCount) {
      deliver(*nodes_[cob - base - 1U]);
      return out;
    }
  }
  ignore(frame, now);
  return out;
}

Bank::InjectResult Bank::inject(std::string_view command, MonoTime now) {
  check_time(now);
  InjectResult res;
  Injection in;
  if (const char* reason = parse_injection(command, in)) {
    res.error = reason;
    std::string text = "err ";
    text.append(reason).append(": ").append(command);
    record_inject(text, now);
    return res;
  }
  res.ok = true;
  std::string text = "ok ";
  text.append(command);
  record_inject(text, now);
  Node& n = node_at(in.node);
  switch (in.verb) {
    case Injection::Verb::kFault:
      run(n, now, res.frames, [code = in.code, now](Drive& d) {
        d.inject_fault(code, now);
        return std::vector<cc::Frame>{};
      });
      break;
    case Injection::Verb::kTorqueLoss:
      run(n, now, res.frames, [now](Drive& d) {
        d.inject_torque_loss(now);
        return std::vector<cc::Frame>{};
      });
      break;
    case Injection::Verb::kStuck:
      run(n, now, res.frames, [flag = in.flag](Drive& d) {
        d.inject_stuck_joint(flag);
        return std::vector<cc::Frame>{};
      });
      break;
    case Injection::Verb::kHeartbeatMute:
      n.muted = in.flag;
      break;
  }
  return res;
}

}  // namespace vdrive_bank
