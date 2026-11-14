// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <compare>
#include <cstdint>
#include <stdexcept>

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

// Checked int64 arithmetic. Wrapping would turn a far-future deadline into a past one (or the
// reverse), so overflow throws instead. Only reachable with values near +/-292 years.
namespace detail {
constexpr std::int64_t checked_add(std::int64_t a, std::int64_t b) {
  std::int64_t r{};
  if (__builtin_add_overflow(a, b, &r)) throw std::overflow_error("time arithmetic overflow");
  return r;
}
constexpr std::int64_t checked_sub(std::int64_t a, std::int64_t b) {
  std::int64_t r{};
  if (__builtin_sub_overflow(a, b, &r)) throw std::overflow_error("time arithmetic overflow");
  return r;
}
constexpr std::int64_t checked_mul(std::int64_t a, std::int64_t b) {
  std::int64_t r{};
  if (__builtin_mul_overflow(a, b, &r)) throw std::overflow_error("time arithmetic overflow");
  return r;
}
}  // namespace detail

constexpr MonoTime operator+(MonoTime t, Duration d) {
  return MonoTime{detail::checked_add(t.ns, d.ns)};
}
constexpr MonoTime operator-(MonoTime t, Duration d) {
  return MonoTime{detail::checked_sub(t.ns, d.ns)};
}
constexpr Duration operator-(MonoTime a, MonoTime b) {
  return Duration{detail::checked_sub(a.ns, b.ns)};
}
constexpr Duration milliseconds(std::int64_t v) {
  return Duration{detail::checked_mul(v, 1'000'000)};
}
constexpr Duration microseconds(std::int64_t v) { return Duration{detail::checked_mul(v, 1'000)}; }

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
