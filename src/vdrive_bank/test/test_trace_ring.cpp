// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>

#include "vdrive_bank/trace_ring.hpp"

namespace {

using runtime_core::MonoTime;
using vdrive_bank::BankRecord;
using vdrive_bank::TraceRecord;
using vdrive_bank::TraceRing;
using vdrive_bank::TraceWriter;
namespace trace = vdrive_bank::trace;

TraceRecord numbered(std::uint64_t n) {
  return trace::Exit{MonoTime{static_cast<std::int64_t>(n)}, 0, 0};
}

void expect_has(const std::string& json, const std::string& part) {
  EXPECT_NE(json.find(part), std::string::npos) << json << "\n  lacks " << part;
}

BankRecord bank_record(BankRecord::Kind kind) {
  BankRecord r{};
  r.kind = kind;
  r.node = 3;
  r.at = MonoTime{42};
  return r;
}

TEST(TraceRecord, EveryKindSerializes) {
  {
    BankRecord r = bank_record(BankRecord::Kind::kDrive);
    r.drive.kind = drive_core::DriveEventKind::kFault;
    r.drive.fault_code = 0x8611;
    const std::string j = vdrive_bank::to_json(r);
    expect_has(j, R"("event":"drive")");
    expect_has(j, R"("t_ns":42)");
    expect_has(j, R"("node":3)");
    expect_has(j, R"("kind":"fault")");
    expect_has(j, R"("fault_code":34321)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kNmt);
    r.nmt_state = 0x05;
    expect_has(vdrive_bank::to_json(r), R"("event":"nmt","t_ns":42,"node":3,"state":5)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kHeartbeat);
    r.nmt_state = 0x7F;
    r.cob_id = 0x703;
    expect_has(vdrive_bank::to_json(r),
               R"("event":"heartbeat","t_ns":42,"node":3,"state":127,"cob_id":1795)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kEmcySent);
    r.emcy_code = 0x6100;
    r.nmt_state = 0x7F;
    r.cob_id = 0x83;
    expect_has(vdrive_bank::to_json(r), R"("event":"emcy_sent","t_ns":42,"node":3,"code":24832)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kEmcySuppressed);
    r.emcy_code = 0x6100;
    r.nmt_state = 0x04;
    expect_has(vdrive_bank::to_json(r),
               R"("event":"emcy_suppressed","t_ns":42,"node":3,"code":24832,"state":4)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kIgnored);
    r.cob_id = 0x123;
    expect_has(vdrive_bank::to_json(r), R"("event":"ignored","t_ns":42,"cob_id":291)");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kInject);
    std::strcpy(r.text, "err bad node: fault \"9\" 1\\");
    expect_has(vdrive_bank::to_json(r),
               R"("event":"inject","t_ns":42,"text":"err bad node: fault \"9\" 1\\")");
  }
  {
    BankRecord r = bank_record(BankRecord::Kind::kOverload);
    expect_has(vdrive_bank::to_json(r), R"("event":"bank_overload")");
  }
  expect_has(vdrive_bank::to_json(trace::Start{MonoTime{1}, MonoTime{2}, {1, 2, 3, 4, 5, 6}}),
             R"({"event":"start","t_ns":1,"t0_ns":2,"boot_ids":[1,2,3,4,5,6]})");
  expect_has(vdrive_bank::to_json(trace::Overload{MonoTime{5}, 3, 0x6100}),
             R"({"event":"overload","t_ns":5,"consecutive_skipped":3,"error_code":24832})");
  {
    trace::LoopSnapshot s{MonoTime{9}, {}, {}, 7};
    s.loop.releases_planned = 10;
    s.loop.releases_skipped = 2;
    s.ignored.heartbeat = 4;
    const std::string j = vdrive_bank::to_json(s);
    expect_has(j, R"("event":"loop","t_ns":9)");
    expect_has(j, R"("releases_planned":10)");
    expect_has(j, R"("releases_skipped":2)");
    expect_has(j, R"("rx_budget_hits":0)");
    expect_has(j, R"("ignored":{"nmt_unknown_node":0,"emcy":0,"pdo":0,"sdo":0,"heartbeat":4)");
    expect_has(j, R"("trace_overflow":7)");
  }
  {
    trace::DispatchSnapshot s{MonoTime{9}, 2, {}, 0};
    s.counts.sdo = 12;
    s.counts.malformed = 1;
    const std::string j = vdrive_bank::to_json(s);
    expect_has(j, R"("event":"dispatch","t_ns":9,"node":2)");
    expect_has(j, R"("sdo":12)");
    expect_has(j, R"("malformed":1)");
    expect_has(j, R"("trace_overflow":0)");
  }
  {
    trace::IoSnapshot s{MonoTime{9}, {}, 0};
    s.io.rx = 5;
    s.io.rx_rejected[static_cast<std::size_t>(socketcan_io::RxClass::kFd)] = 2;
    s.io.tx_enobufs = 1;
    const std::string j = vdrive_bank::to_json(s);
    expect_has(j, R"("event":"io","t_ns":9,"rx":5)");
    expect_has(j, R"("rx_rejected":{"rtr":0,"error":0,"fd":2,"extended_id":0,"bad_length":0})");
    expect_has(j, R"("rx_error":0)");
    expect_has(j, R"("rx_reject_budget_hits":0)");
    expect_has(j, R"("tx_enobufs":1)");
  }
  expect_has(vdrive_bank::to_json(trace::Exit{MonoTime{11}, 0, 15}),
             R"({"event":"exit","t_ns":11,"trace_overflow":0,"signal":15})");
}

TEST(TraceRing, FullRingCountsOverflowAndNeverBlocks) {
  TraceRing ring;
  for (std::uint64_t i = 0; i < TraceRing::kCapacity; ++i) {
    ASSERT_TRUE(ring.push(numbered(i)));
  }
  EXPECT_FALSE(ring.push(numbered(9999)));
  EXPECT_FALSE(ring.push(numbered(9999)));
  EXPECT_EQ(ring.overflow(), 2U);
  // FIFO, and the dropped records are gone.
  auto first = ring.pop();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(std::get<trace::Exit>(*first).at.ns, 0);
  EXPECT_TRUE(ring.push(numbered(4096)));
  std::int64_t expected = 1;
  while (auto r = ring.pop()) {
    EXPECT_EQ(std::get<trace::Exit>(*r).at.ns, expected++);
  }
  EXPECT_EQ(expected, 4097);
  EXPECT_EQ(ring.overflow(), 2U);
}

TEST(TraceRing, WriterBlockedDoesNotBlockProducer) {
  TraceRing ring;
  std::mutex m;
  std::condition_variable cv;
  bool release = false;
  std::uint64_t sunk = 0;
  TraceWriter writer(ring, [&](const TraceRecord&) {
    std::unique_lock lock(m);
    cv.wait(lock, [&] { return release; });  // The writer stalls on the first record.
    ++sunk;
  });
  writer.start();
  constexpr std::uint64_t kPushes = 10'000;
  std::uint64_t accepted = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (std::uint64_t i = 0; i < kPushes; ++i) {
    accepted += ring.push(numbered(i)) ? 1U : 0U;
  }
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
  // The stalled writer holds at most one record outside the ring.
  EXPECT_GE(ring.overflow(), kPushes - TraceRing::kCapacity - 1U);
  EXPECT_EQ(accepted + ring.overflow(), kPushes);
  {
    std::lock_guard lock(m);
    release = true;
  }
  cv.notify_all();
  writer.stop();
  EXPECT_EQ(sunk, accepted);
  EXPECT_EQ(writer.written(), accepted);
}

}  // namespace
