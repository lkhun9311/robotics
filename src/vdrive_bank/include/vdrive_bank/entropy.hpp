// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Entropy for the first boot_id (0x2013) of each drive (B5).
// A constant seed would give the same first execution tag on every process start, so the process
// draws one per node at start.
namespace vdrive_bank {

inline constexpr std::size_t kNodeCount = 6;  // Node IDs 1..6.

class EntropySource {
 public:
  virtual ~EntropySource() = default;
  virtual std::uint32_t draw() = 0;
};

// getrandom(2), four bytes per draw.
// Throws std::system_error if the kernel refuses.
class GetrandomEntropy final : public EntropySource {
 public:
  std::uint32_t draw() override;
};

// One boot_id per node, in node order: nonzero and distinct within the process.
// A zero or a duplicate draw is drawn again.
// Throws std::runtime_error if the source keeps producing values that cannot be used (a broken
// source must not hang process start).
std::array<std::uint32_t, kNodeCount> draw_boot_ids(EntropySource& source);

}  // namespace vdrive_bank
