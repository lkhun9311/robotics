// SPDX-License-Identifier: Apache-2.0
#include "runtime_core/capability.hpp"

namespace runtime_core {

namespace {

bool met(Support s) { return s == Support::kSupported || s == Support::kProvidedByHost; }

void require(bool required, Support s, Capability c, std::vector<Missing>& out) {
  if (required && !met(s)) out.push_back(Missing{c, s});
}

}  // namespace

const char* to_string(Capability capability) {
  switch (capability) {
    case Capability::kDeviceSideExpiry:
      return "DEVICE_SIDE_EXPIRY";
    case Capability::kExecutionGeneration:
      return "EXECUTION_GENERATION";
    case Capability::kLocalStop:
      return "LOCAL_STOP";
    case Capability::kApplyAck:
      return "APPLY_ACK";
    case Capability::kSourceTimestamp:
      return "SOURCE_TIMESTAMP";
  }
  return "UNKNOWN";
}

const char* to_string(Support support) {
  switch (support) {
    case Support::kSupported:
      return "SUPPORTED";
    case Support::kProvidedByHost:
      return "PROVIDED_BY_HOST";
    case Support::kNotApplicable:
      return "NOT_APPLICABLE";
    case Support::kUnknown:
      return "UNKNOWN";
    case Support::kUnsupported:
      return "UNSUPPORTED";
  }
  return "UNKNOWN";
}

std::vector<Missing> missing_capabilities(const Requirements& required,
                                          const DeviceCapabilities& device) {
  std::vector<Missing> missing;
  require(required.device_side_expiry, device.device_side_expiry, Capability::kDeviceSideExpiry,
          missing);
  require(required.execution_generation, device.execution_generation,
          Capability::kExecutionGeneration, missing);
  require(required.local_stop, device.local_stop, Capability::kLocalStop, missing);
  require(required.apply_ack, device.apply_ack, Capability::kApplyAck, missing);
  require(required.source_timestamp, device.source_timestamp, Capability::kSourceTimestamp,
          missing);
  return missing;
}

DeviceCapabilities direct_backend_profile() {
  return {Support::kProvidedByHost, Support::kSupported, Support::kSupported, Support::kSupported,
          Support::kSupported};
}

DeviceCapabilities virtual_canopen_profile() {
  return {Support::kSupported, Support::kSupported, Support::kSupported, Support::kSupported,
          Support::kSupported};
}

// As documented by CiA 402 without project extensions. Quick-stop behaviour and source
// timestamps depend on the drive and its configuration, so they stay unknown until checked.
DeviceCapabilities generic_cia402_profile() {
  return {Support::kUnsupported, Support::kUnsupported, Support::kUnknown, Support::kUnsupported,
          Support::kUnknown};
}

DeviceCapabilities modbus_io_profile() {
  return {Support::kUnsupported, Support::kUnsupported, Support::kNotApplicable,
          Support::kUnsupported, Support::kUnsupported};
}

}  // namespace runtime_core
