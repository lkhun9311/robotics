// SPDX-License-Identifier: Apache-2.0
#include <dlfcn.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "arm_hw/arm_system.hpp"
#include "canopen_backend/canopen_backend.hpp"
#include "canopen_backend/socket_transport.hpp"
#include "canopen_backend/worker.hpp"
#include "canopen_codec/codec.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "socketcan_io/socket.hpp"

namespace arm_hw {

namespace cb = canopen_backend;
namespace hc = hal_contract;
namespace rc = runtime_core;

namespace {

std::atomic<testing::TransportFactory> g_factory{nullptr};
std::atomic<bool> g_pinned{false};
// Any object of this library: dladdr names the library that holds it.
const char g_anchor = 0;

// Marks this library RTLD_NODELETE: a later dlclose leaves it mapped. The handle is never closed.
void pin_this_library() {
  Dl_info info{};
  if (::dladdr(&g_anchor, &info) == 0 || info.dli_fname == nullptr) return;
  if (::dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD | RTLD_NODELETE) != nullptr) {
    g_pinned.store(true);
  }
}

}  // namespace

CanopenArmHw::~CanopenArmHw() {
  // A worker still running now may outlive this object (Worker::~Worker detaches it after 1 s);
  // its code is in this library, which must then stay mapped.
  if (worker_ != nullptr && worker_->running()) pin_this_library();
  drop_cycle();
}

bool CanopenArmHw::library_pinned() { return g_pinned.load(); }

std::unique_ptr<hc::Backend> CanopenArmHw::make_backend(const Params& params, std::string* error) {
  std::unique_ptr<cb::Transport> transport;
  if (const testing::TransportFactory factory = g_factory.load()) {
    transport = factory(params.can_interface, error);
  } else {
    std::optional<socketcan_io::CanSocket> socket =
        socketcan_io::CanSocket::open(params.can_interface, error);
    if (socket) transport = std::make_unique<cb::SocketTransport>(std::move(*socket));
  }
  if (!transport) {
    if (error->empty()) *error = "no transport on " + params.can_interface;
    return nullptr;
  }
  cb::CanopenConfig config;
  for (std::size_t i = 0; i < hc::kJoints; ++i) {
    const std::optional<canopen_codec::NodeId> id =
        canopen_codec::NodeId::make(params.node_ids.at(i));
    if (!id) {  // parse_node_ids accepts 1..127 only.
      *error = "node_ids: not a CANopen node ID";
      return nullptr;
    }
    config.session.nodes.at(i) = *id;
  }
  config.session.bus_period = rc::milliseconds(params.timing.bus_period_ms);
  config.session.lease = rc::milliseconds(params.timing.lease_ms);
  config.session.max_feedback_age = rc::milliseconds(params.timing.max_feedback_age_ms);
  config.profile_limit = params.profile_limit;
  auto worker = std::make_unique<cb::Worker>(config, std::move(transport));
  if (!worker->start(error)) return nullptr;
  worker_ = worker.get();
  return worker;
}

bool CanopenArmHw::backend_running() const { return worker_ != nullptr && worker_->running(); }

bool CanopenArmHw::stop_backend(rc::Duration timeout) {
  return worker_ == nullptr || worker_->shutdown(timeout);
}

namespace testing {
void set_transport_factory(TransportFactory factory) { g_factory.store(factory); }
}  // namespace testing

}  // namespace arm_hw

extern "C" void arm_hw_testing_set_transport_factory(arm_hw::testing::TransportFactory factory) {
  arm_hw::testing::set_transport_factory(factory);
}

PLUGINLIB_EXPORT_CLASS(arm_hw::CanopenArmHw, hardware_interface::SystemInterface)
