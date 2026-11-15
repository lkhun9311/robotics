// SPDX-License-Identifier: Apache-2.0
#include "canopen_backend/canopen_backend.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace canopen_backend {

namespace cc = canopen_codec;
namespace cm = canopen_master;
namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

int rank(rc::Support s) {
  switch (s) {
    case rc::Support::kUnsupported:
      return 0;
    case rc::Support::kUnknown:
      return 1;
    case rc::Support::kNotApplicable:
      return 2;
    case rc::Support::kProvidedByHost:
      return 3;
    case rc::Support::kSupported:
      return 4;
  }
  return 0;
}

cm::MasterConfig master_config(const CanopenConfig& c) {
  cm::MasterConfig m;
  m.nodes.assign(c.session.nodes.begin(), c.session.nodes.end());
  m.sync_period = c.session.bus_period;
  m.heartbeat_timeout = c.heartbeat_timeout;
  m.boot_timeout = c.boot_timeout;
  m.sdo_timeout = c.sdo_timeout;
  return m;
}

std::unique_ptr<Transport> require(std::unique_ptr<Transport> transport) {
  if (!transport) {
    throw std::invalid_argument("canopen_backend: a transport is required");
  }
  return transport;
}

constexpr const char* kRefusedWhileArmed =
    "configure refused while armed, arming or deactivating: deactivate first (its "
    "Pre-operational would stop enabled drives untracked)";

}  // namespace

rc::Support weaker(rc::Support a, rc::Support b) { return rank(a) <= rank(b) ? a : b; }

rc::DeviceCapabilities weaker(const rc::DeviceCapabilities& a, const rc::DeviceCapabilities& b) {
  return rc::DeviceCapabilities{
      weaker(a.device_side_expiry, b.device_side_expiry),
      weaker(a.execution_generation, b.execution_generation),
      weaker(a.local_stop, b.local_stop),
      weaker(a.apply_ack, b.apply_ack),
      weaker(a.source_timestamp, b.source_timestamp),
  };
}

CanopenBackend::CanopenBackend(CanopenConfig config, std::unique_ptr<Transport> transport)
    : config_(config), transport_(require(std::move(transport))) {
  try {
    master_.emplace(master_config(config_));
    session_.emplace(*master_, config_.session);
  } catch (const std::invalid_argument& e) {
    // C20: reported by the configure op, with the allowed values, instead of thrown.
    session_.reset();
    master_.reset();
    config_error_ = std::string("canopen_backend: configuration refused: ") + e.what();
  }
}

const cm::VdriveSession& CanopenBackend::session() const {
  if (!session_) throw std::logic_error(config_error_);
  return *session_;
}

const cm::Master& CanopenBackend::master() const {
  if (!master_) throw std::logic_error(config_error_);
  return *master_;
}

void CanopenBackend::send_all(const std::vector<cc::Frame>& frames) {
  for (const cc::Frame& f : frames) transport_->send(f);
}

std::size_t CanopenBackend::step(rc::MonoTime now, std::size_t max_frames) {
  now_ = now;
  std::size_t handled = 0;
  const auto drain = [&] {
    while (handled < max_frames) {
      const std::vector<cc::Frame> received = transport_->poll();
      if (received.empty()) return;
      handled += received.size();
      if (!session_) continue;  // A refused configuration: received frames are dropped.
      for (const cc::Frame& f : received) send_all(session_->on_frame(f, now));
    }
  };
  drain();
  if (session_) send_all(session_->tick(now));
  // The replies to this tick, the TPDOs answering its SYNC among them, reach poll() now.
  drain();
  return handled;
}

void CanopenBackend::cancel(std::uint64_t op_id, const char* why, rc::MonoTime now) {
  now_ = now;
  if (session_) session_->cancel(op_id, why, now);
}

const char* CanopenBackend::op_error(std::uint64_t op_id) const {
  if (const Refusal* r = refusal(op_id)) return r->error;
  return session_ ? session_->op_error(op_id) : nullptr;
}

void CanopenBackend::set_event_observer(cm::VdriveSession::EventObserver observer) {
  if (session_) session_->set_event_observer(std::move(observer));
}

// ---- ops ----

const CanopenBackend::Refusal* CanopenBackend::refusal(std::uint64_t op_id) const {
  for (const Refusal& r : refusals_) {
    if (r.op_id != 0 && r.op_id == op_id) return &r;
  }
  return nullptr;
}

void CanopenBackend::refuse(const hc::OpTicket& ticket, const char* error) {
  refusals_[refusals_next_] = Refusal{ticket.op_id, error};
  refusals_next_ = (refusals_next_ + 1) % kRefusalRing;
}

bool CanopenBackend::configure_unsafe() const {
  if (!session_ || session_->inhibited()) return false;  // The inhibit tracks every stop (C17).
  switch (session_->phase()) {
    case cm::SessionPhase::kArming:
    case cm::SessionPhase::kArmed:
    case cm::SessionPhase::kDeactivating:
      return true;
    default:
      return false;
  }
}

void CanopenBackend::configure(const hc::OpTicket& ticket, rc::MonoTime now) {
  now_ = now;
  if (!session_) {
    refuse(ticket, config_error_.c_str());
  } else if (configure_unsafe()) {
    refuse(ticket, kRefusedWhileArmed);
  } else {
    session_->configure(ticket);
  }
}

rc::DeviceCapabilities CanopenBackend::capabilities() const {
  if (!session_) return rc::DeviceCapabilities{};  // every guarantee kUnknown
  const rc::DeviceCapabilities probed = session_->capabilities();
  return config_.profile_limit ? weaker(probed, *config_.profile_limit) : probed;
}

void CanopenBackend::activate(const hc::OpTicket& ticket, rc::MonoTime now) {
  now_ = now;
  if (!session_) return refuse(ticket, config_error_.c_str());
  session_->activate(ticket);
}

void CanopenBackend::arm(const hc::OpTicket& ticket, const hc::ArmRequest& request,
                         rc::MonoTime now) {
  now_ = now;
  if (!session_) return refuse(ticket, config_error_.c_str());
  last_arm_ = request;
  session_->arm(ticket, request);
}

void CanopenBackend::submit(const hc::ApprovedCommand& command, rc::MonoTime /*now*/) {
  if (session_) session_->offer(command);
}

hc::Feedback CanopenBackend::poll(rc::MonoTime /*now*/) {
  return session_ ? session_->feedback() : hc::Feedback{};
}

void CanopenBackend::deactivate(const hc::OpTicket& ticket, rc::MonoTime now) {
  now_ = now;
  if (!session_) return refuse(ticket, config_error_.c_str());
  session_->deactivate(ticket);
}

void CanopenBackend::on_error_nonblocking() {
  if (session_) session_->inhibit(now_);
}

hc::OpState CanopenBackend::op_state(std::uint64_t op_id) const {
  if (refusal(op_id) != nullptr) return hc::OpState::kFailed;
  return session_ ? session_->op_state(op_id) : hc::OpState::kUnknown;
}

std::optional<hc::OpResult> CanopenBackend::wait(std::uint64_t op_id, rc::Duration /*timeout*/) {
  if (const Refusal* r = refusal(op_id)) return hc::OpResult{false, r->error, {}};
  if (!session_) return hc::OpResult{false, "unknown operation", {}};
  switch (session_->op_state(op_id)) {
    case hc::OpState::kPending:
      return std::nullopt;  // Logic time: nothing to wait on.
    case hc::OpState::kSucceeded:
      return hc::OpResult{true, "", {}};
    case hc::OpState::kFailed: {
      const char* error = session_->op_error(op_id);
      return hc::OpResult{false, error != nullptr ? error : "failed", {}};
    }
    case hc::OpState::kUnknown:
      break;
  }
  return hc::OpResult{false, "unknown operation", {}};
}

}  // namespace canopen_backend
