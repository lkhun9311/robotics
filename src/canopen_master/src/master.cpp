// SPDX-License-Identifier: Apache-2.0
#include "canopen_master/master.hpp"

#include <stdexcept>
#include <utility>

namespace canopen_master {

namespace cc = canopen_codec;
using runtime_core::Duration;
using runtime_core::MonoTime;

namespace {

constexpr std::uint16_t kNmtCobId = 0x000;
constexpr std::uint16_t kHeartbeatBase = 0x700;
constexpr std::uint16_t kSdoResponseBase = 0x580;

MasterEventKind sdo_kind(SdoStatus status) {
  switch (status) {
    case SdoStatus::kOk:
      return MasterEventKind::kSdoOk;
    case SdoStatus::kAbort:
      return MasterEventKind::kSdoAbort;
    case SdoStatus::kTimeout:
      return MasterEventKind::kSdoTimeout;
    case SdoStatus::kProtocol:
      break;
  }
  return MasterEventKind::kSdoProtocol;
}

}  // namespace

Master::Master(MasterConfig config) : config_(std::move(config)) {
  if (config_.nodes.empty()) {
    throw std::invalid_argument("canopen_master: no nodes");
  }
  if (config_.boot_timeout.ns <= 0 || config_.sdo_timeout.ns <= 0) {
    throw std::invalid_argument("canopen_master: boot and SDO timeouts must be positive");
  }
  slot_.fill(-1);
  nodes_.reserve(config_.nodes.size());
  for (const cc::NodeId id : config_.nodes) {
    if (slot_.at(id.value()) != -1) {
      throw std::invalid_argument("canopen_master: duplicate node");
    }
    slot_.at(id.value()) = static_cast<std::int16_t>(nodes_.size());
    nodes_.push_back(Node{id, HeartbeatConsumer{config_.heartbeat_timeout, config_.boot_timeout},
                          SdoClient{id, config_.sdo_timeout}, std::nullopt});
  }
}

Master::Node* Master::find(cc::NodeId node) {
  const std::int16_t slot = slot_.at(node.value());
  return slot < 0 ? nullptr : &nodes_.at(static_cast<std::size_t>(slot));
}

const Master::Node* Master::find(cc::NodeId node) const {
  const std::int16_t slot = slot_.at(node.value());
  return slot < 0 ? nullptr : &nodes_.at(static_cast<std::size_t>(slot));
}

// The configured node whose COB-ID for the service at `base` is `cob`.
Master::Node* Master::find(std::uint16_t cob, std::uint16_t base) {
  if (cob <= base || cob > base + cc::NodeId::kMax) {
    return nullptr;
  }
  const auto id = cc::NodeId::make(static_cast<std::uint8_t>(cob - base));
  return id ? find(*id) : nullptr;
}

void Master::emit(MasterEventKind kind, std::uint8_t node, EventDetail detail) {
  events_.push_back(MasterEvent{kind, node, now_, detail});
}

std::vector<MasterEvent> Master::drain_events() { return std::exchange(events_, {}); }

bool Master::nmt(cc::NmtCommand command, cc::NodeId node) {
  if (find(node) == nullptr) {
    return false;
  }
  pending_nmt_.push_back(cc::NmtMessage{command, node});
  return true;
}

bool Master::sdo_upload(cc::NodeId node, std::uint16_t index, std::uint8_t sub, SdoDone done) {
  Node* n = find(node);
  if (n == nullptr || !n->sdo.upload(index, sub, std::move(done))) {
    ++counts_.sdo_refused;
    return false;
  }
  return true;
}

bool Master::sdo_download(cc::NodeId node, std::uint16_t index, std::uint8_t sub,
                          std::span<const std::uint8_t> bytes, SdoDone done) {
  Node* n = find(node);
  if (n == nullptr || !n->sdo.download(index, sub, bytes, std::move(done))) {
    ++counts_.sdo_refused;
    return false;
  }
  return true;
}

bool Master::set_heartbeat_consumer(cc::NodeId node, Duration consumer_time) {
  if (find(node) == nullptr) {
    return false;
  }
  pending_consumer_.emplace_back(node, consumer_time);
  return true;
}

std::optional<cc::NmtState> Master::nmt_state(cc::NodeId node) const {
  const Node* n = find(node);
  return n == nullptr ? std::nullopt : n->state;
}

std::optional<HeartbeatStatus> Master::heartbeat_status(cc::NodeId node) const {
  const Node* n = find(node);
  if (n == nullptr) {
    return std::nullopt;
  }
  return n->heartbeat.status();
}

void Master::apply_commands(MonoTime now, std::vector<cc::Frame>& out) {
  // Nothing called from here queues commands, so the vectors are iterated in place and keep
  // their capacity.
  for (const auto& [id, time] : pending_consumer_) {
    find(id)->heartbeat.set_consumer_time(time, now);
  }
  pending_consumer_.clear();
  for (const cc::NmtMessage& message : pending_nmt_) {
    out.push_back(cc::encode_nmt(message));
    if (message.command == cc::NmtCommand::kResetNode ||
        message.command == cc::NmtCommand::kResetCommunication) {
      Node* n = find(*message.target);
      n->heartbeat.start(now);
      n->state.reset();
    }
  }
  pending_nmt_.clear();
  for (Node& n : nodes_) {
    if (auto frame = n.sdo.start(now)) {
      out.push_back(*frame);
    }
  }
}

void Master::complete(SdoCompletion completion, std::vector<cc::Frame>& out) {
  if (completion.abort) {
    out.push_back(*completion.abort);
  }
  if (completion.result.status == SdoStatus::kProtocol) {
    ++counts_.sdo_protocol_errors;
  }
  emit(sdo_kind(completion.result.status), completion.result.node, completion.result);
  if (completion.done) {
    pending_done_.push_back(Done{std::move(completion.done), completion.result});
  }
}

void Master::expire(MonoTime now, std::vector<cc::Frame>& out) {
  for (Node& n : nodes_) {
    switch (n.heartbeat.expire(now)) {
      case HeartbeatChange::kBootTimeout:
        emit(MasterEventKind::kBootTimeout, n.id.value());
        break;
      case HeartbeatChange::kLost:
        emit(MasterEventKind::kHeartbeatLost, n.id.value());
        break;
      default:
        break;
    }
    if (auto completion = n.sdo.expire(now)) {
      complete(std::move(*completion), out);
    }
  }
}

void Master::handle_heartbeat(Node& node, const cc::Frame& frame) {
  const auto heartbeat = cc::decode_heartbeat(frame);
  if (!heartbeat) {
    ++counts_.malformed;
    return;
  }
  if (node.heartbeat.on_heartbeat(now_) == HeartbeatChange::kResumed) {
    emit(MasterEventKind::kHeartbeatResumed, node.id.value());
  }
  if (heartbeat->state == cc::NmtState::kBootUp) {
    node.state = cc::NmtState::kBootUp;
    emit(MasterEventKind::kBoot, node.id.value());
  } else if (node.state != heartbeat->state) {
    node.state = heartbeat->state;
    emit(MasterEventKind::kState, node.id.value(), StateDetail{heartbeat->state});
  }
}

void Master::handle(const cc::Frame& frame, std::vector<cc::Frame>& out) {
  const std::uint16_t cob = frame.cob_id;
  if (cob == kNmtCobId) {
    ++counts_.foreign;
    return;
  }
  if (cob > kHeartbeatBase && cob <= kHeartbeatBase + cc::NodeId::kMax) {
    if (Node* n = find(cob, kHeartbeatBase)) {
      handle_heartbeat(*n, frame);
    } else {
      ++counts_.foreign;
    }
    return;
  }
  if (cob > kSdoResponseBase && cob <= kSdoResponseBase + cc::NodeId::kMax) {
    Node* n = find(cob, kSdoResponseBase);
    if (n == nullptr) {
      ++counts_.foreign;
    } else if (auto completion = n->sdo.on_response(frame)) {
      complete(std::move(*completion), out);
    } else {
      ++counts_.sdo_unexpected;
    }
    return;
  }
  ++counts_.unregistered_cob;
}

void Master::run_callbacks() {
  // Taken out before any is called: a callback only queues commands, but the list must not be
  // the one a later completion appends to.
  auto done = std::exchange(pending_done_, {});
  for (Done& d : done) {
    d.done(d.result);
  }
}

std::vector<cc::Frame> Master::step(const cc::Frame* frame, MonoTime now) {
  if (in_step_) {
    throw std::logic_error("canopen_master: on_frame or tick called from an SDO callback");
  }
  in_step_ = true;
  now_ = now;
  std::vector<cc::Frame> out;
  if (!started_) {
    started_ = true;
    for (Node& n : nodes_) {
      n.heartbeat.start(now);
    }
  }
  apply_commands(now, out);
  expire(now, out);
  if (frame != nullptr) {
    handle(*frame, out);
  }
  try {
    run_callbacks();
  } catch (...) {
    in_step_ = false;
    throw;
  }
  apply_commands(now, out);
  in_step_ = false;
  return out;
}

std::vector<cc::Frame> Master::on_frame(const cc::Frame& frame, MonoTime now) {
  return step(&frame, now);
}

std::vector<cc::Frame> Master::tick(MonoTime now) { return step(nullptr, now); }

}  // namespace canopen_master
