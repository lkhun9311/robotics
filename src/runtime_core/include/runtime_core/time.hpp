// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <compare>
#include <cstdint>

namespace runtime_core {

// Host CLOCK_MONOTONIC in integer nanoseconds. Never mixed with simulation time.
struct MonoTime {
  std::int64_t ns{0};
  auto operator<=>(const MonoTime&) const = default;
};

struct Duration {
  std::int64_t ns{0};
  auto operator<=>(const Duration&) const = default;
};

constexpr MonoTime operator+(MonoTime t, Duration d) { return MonoTime{t.ns + d.ns}; }
constexpr MonoTime operator-(MonoTime t, Duration d) { return MonoTime{t.ns - d.ns}; }
constexpr Duration operator-(MonoTime a, MonoTime b) { return Duration{a.ns - b.ns}; }
constexpr Duration milliseconds(std::int64_t v) { return Duration{v * 1'000'000}; }
constexpr Duration microseconds(std::int64_t v) { return Duration{v * 1'000}; }

class Clock {
 public:
  virtual ~Clock() = default;
  virtual MonoTime now() const = 0;
};

class SystemMonotonicClock final : public Clock {
 public:
  MonoTime now() const override;
};

// Deterministic clock for logic-time tests. It can only move forward.
class FakeClock final : public Clock {
 public:
  explicit FakeClock(MonoTime start = MonoTime{}) : now_(start) {}
  MonoTime now() const override { return now_; }
  void advance(Duration d);

 private:
  MonoTime now_;
};

}  // namespace runtime_core
