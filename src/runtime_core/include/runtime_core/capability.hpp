// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

namespace runtime_core {

// How a backend stands on one guarantee (spec §4). A HAL cannot create a guarantee the device
// lacks; activation is refused instead of silently weakening the contract.
enum class Support {
  kSupported,       // the device enforces it
  kProvidedByHost,  // same process: the host guard enforces it directly
  kNotApplicable,   // meaningless for this device (e.g. STOP on a non-motion I/O)
  kUnknown,         // depends on device configuration not yet verified
  kUnsupported,     // the device does not offer it
};

struct DeviceCapabilities {
  Support device_side_expiry{Support::kUnknown};
  Support execution_generation{Support::kUnknown};
  Support local_stop{Support::kUnknown};
  Support apply_ack{Support::kUnknown};
  Support source_timestamp{Support::kUnknown};
};

struct Requirements {
  bool device_side_expiry{false};
  bool execution_generation{false};
  bool local_stop{false};
  bool apply_ack{false};
  bool source_timestamp{false};
};

enum class Capability {
  kDeviceSideExpiry,
  kExecutionGeneration,
  kLocalStop,
  kApplyAck,
  kSourceTimestamp,
};

struct Missing {
  Capability capability;
  Support support;
  bool operator==(const Missing&) const = default;
};

const char* to_string(Capability capability);
const char* to_string(Support support);

// A requirement is met only by kSupported or kProvidedByHost.
std::vector<Missing> missing_capabilities(const Requirements& required,
                                          const DeviceCapabilities& device);

DeviceCapabilities direct_backend_profile();
DeviceCapabilities virtual_canopen_profile();
DeviceCapabilities generic_cia402_profile();
DeviceCapabilities modbus_io_profile();

}  // namespace runtime_core
