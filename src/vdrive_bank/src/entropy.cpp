// SPDX-License-Identifier: Apache-2.0
#include "vdrive_bank/entropy.hpp"

#include <sys/random.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace vdrive_bank {

std::uint32_t GetrandomEntropy::draw() {
  unsigned char bytes[sizeof(std::uint32_t)];
  std::size_t have = 0;
  while (have < sizeof(bytes)) {
    const ssize_t n = ::getrandom(bytes + have, sizeof(bytes) - have, 0);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::system_error(errno, std::generic_category(), "getrandom");
    }
    have += static_cast<std::size_t>(n);
  }
  std::uint32_t value = 0;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

std::array<std::uint32_t, kNodeCount> draw_boot_ids(EntropySource& source) {
  // A working source needs a redraw with probability about 6 * 6 / 2^32 per ID; this bound is
  // only ever reached by a broken one.
  constexpr int kMaxDraws = 64 * static_cast<int>(kNodeCount);
  std::array<std::uint32_t, kNodeCount> ids{};
  std::size_t filled = 0;
  for (int draws = 0; filled < kNodeCount; ++draws) {
    if (draws == kMaxDraws) {
      throw std::runtime_error("entropy source yields no usable boot_id");
    }
    const std::uint32_t v = source.draw();
    const auto end = ids.begin() + static_cast<std::ptrdiff_t>(filled);
    if (v != 0U && std::find(ids.begin(), end, v) == end) {
      ids[filled++] = v;
    }
  }
  return ids;
}

}  // namespace vdrive_bank
