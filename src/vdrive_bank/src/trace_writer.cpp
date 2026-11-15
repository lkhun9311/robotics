// SPDX-License-Identifier: Apache-2.0
#include <cstdio>
#include <cstring>
#include <string_view>
#include <utility>

#include "vdrive_bank/trace_ring.hpp"

namespace vdrive_bank {

namespace {

// Appends "key":value pairs to one JSON object. Only integers and strings occur in the trace.
class Json {
 public:
  Json(std::string_view event, runtime_core::MonoTime at) {
    out_ = R"({"event":")";
    out_.append(event);
    out_ += '"';
    num("t_ns", at.ns);
  }
  template <typename T>
  Json& num(std::string_view key, T value) {
    this->key(key);
    if constexpr (std::is_signed_v<T>) {
      out_ += std::to_string(static_cast<long long>(value));
    } else {
      out_ += std::to_string(static_cast<unsigned long long>(value));
    }
    return *this;
  }
  Json& str(std::string_view key, std::string_view value) {
    this->key(key);
    out_ += '"';
    for (const char c : value) {
      if (c == '"' || c == '\\') {
        out_ += '\\';
        out_ += c;
      } else if (static_cast<unsigned char>(c) < 0x20U) {
        char buf[8];
        std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(c));
        out_ += buf;
      } else {
        out_ += c;
      }
    }
    out_ += '"';
    return *this;
  }
  // A nested object or array written by the caller: `raw` is appended as is.
  Json& raw(std::string_view key, std::string_view raw) {
    this->key(key);
    out_.append(raw);
    return *this;
  }
  std::string done() {
    out_ += '}';
    return std::move(out_);
  }

 private:
  void key(std::string_view k) {
    out_ += ",\"";
    out_.append(k);
    out_ += "\":";
  }
  std::string out_;
};

const char* drive_event_name(drive_core::DriveEventKind k) {
  switch (k) {
    case drive_core::DriveEventKind::kActuated:
      return "actuated";
    case drive_core::DriveEventKind::kDropped:
      return "dropped";
    case drive_core::DriveEventKind::kQuickStop:
      return "quick_stop";
    case drive_core::DriveEventKind::kStopConfirmed:
      return "stop_confirmed";
    case drive_core::DriveEventKind::kStopUnconfirmed:
      return "stop_unconfirmed";
    case drive_core::DriveEventKind::kFault:
      return "fault";
    case drive_core::DriveEventKind::kIntegrity:
      return "integrity";
  }
  return "unknown";
}

// {"a":1,"b":2} from (name, value) pairs.
template <typename... Pairs>
std::string object(const Pairs&... pairs) {
  std::string out = "{";
  bool first = true;
  const auto add = [&](const std::pair<const char*, std::uint64_t>& p) {
    if (!first) {
      out += ',';
    }
    first = false;
    out += '"';
    out += p.first;
    out += "\":";
    out += std::to_string(p.second);
  };
  (add(pairs), ...);
  out += '}';
  return out;
}

using P = std::pair<const char*, std::uint64_t>;

std::string bank_json(const BankRecord& r) {
  switch (r.kind) {
    case BankRecord::Kind::kDrive: {
      const drive_core::DriveEvent& e = r.drive;
      return Json("drive", r.at)
          .num("node", r.node)
          .str("kind", drive_event_name(e.kind))
          .num("seq", e.seq)
          .num("tag", e.tag)
          .num("drop", static_cast<unsigned>(e.drop))
          .num("count", e.count)
          .num("reason", static_cast<unsigned>(e.reason))
          .num("fault_code", e.fault_code)
          .done();
    }
    case BankRecord::Kind::kNmt:
      return Json("nmt", r.at).num("node", r.node).num("state", r.nmt_state).done();
    case BankRecord::Kind::kHeartbeat:
      return Json("heartbeat", r.at)
          .num("node", r.node)
          .num("state", r.nmt_state)
          .num("cob_id", r.cob_id)
          .done();
    case BankRecord::Kind::kEmcySent:
      return Json("emcy_sent", r.at)
          .num("node", r.node)
          .num("code", r.emcy_code)
          .num("state", r.nmt_state)
          .num("cob_id", r.cob_id)
          .done();
    case BankRecord::Kind::kEmcySuppressed:
      return Json("emcy_suppressed", r.at)
          .num("node", r.node)
          .num("code", r.emcy_code)
          .num("state", r.nmt_state)
          .done();
    case BankRecord::Kind::kIgnored:
      return Json("ignored", r.at).num("cob_id", r.cob_id).done();
    case BankRecord::Kind::kInject: {
      const std::string_view text(r.text, strnlen(r.text, sizeof r.text));
      return Json("inject", r.at).str("text", text).done();
    }
    case BankRecord::Kind::kOverload:
      return Json("bank_overload", r.at).num("node", r.node).done();
  }
  return Json("bank_unknown", r.at).num("kind", static_cast<unsigned>(r.kind)).done();
}

struct ToJson {
  std::string operator()(const BankRecord& r) const { return bank_json(r); }
  std::string operator()(const trace::Start& s) const {
    std::string ids = "[";
    for (std::size_t i = 0; i < s.boot_ids.size(); ++i) {
      ids += (i == 0 ? "" : ",") + std::to_string(s.boot_ids[i]);
    }
    ids += ']';
    return Json("start", s.at).num("t0_ns", s.t0.ns).raw("boot_ids", ids).done();
  }
  std::string operator()(const trace::Overload& o) const {
    return Json("overload", o.at)
        .num("consecutive_skipped", o.consecutive_skipped)
        .num("error_code", o.error_code)
        .done();
  }
  std::string operator()(const trace::LoopSnapshot& s) const {
    const LoopCounts& c = s.loop;
    const IgnoredCounts& g = s.ignored;
    return Json("loop", s.at)
        .num("releases_planned", c.releases_planned)
        .num("releases_executed", c.releases_executed)
        .num("releases_skipped", c.releases_skipped)
        .num("consecutive_skipped", c.consecutive_skipped)
        .num("max_consecutive_skipped", c.max_consecutive_skipped)
        .num("rx_budget_hits", c.rx_budget_hits)
        .num("overloads", c.overloads)
        .num("frames_dispatched", c.frames_dispatched)
        .num("wakes", c.wakes)
        .raw("ignored",
             object(P{"nmt_unknown_node", g.nmt_unknown_node}, P{"emcy", g.emcy}, P{"pdo", g.pdo},
                    P{"sdo", g.sdo}, P{"heartbeat", g.heartbeat}, P{"other", g.other}))
        .num("trace_overflow", s.trace_overflow)
        .done();
  }
  std::string operator()(const trace::DispatchSnapshot& s) const {
    const drive_core::DispatchCounts& c = s.counts;
    return Json("dispatch", s.at)
        .num("node", s.node)
        .num("nmt", c.nmt)
        .num("heartbeat", c.heartbeat)
        .num("sync", c.sync)
        .num("emcy", c.emcy)
        .num("sdo", c.sdo)
        .num("pdo", c.pdo)
        .num("gated", c.gated)
        .num("foreign", c.foreign)
        .num("malformed", c.malformed)
        .num("trace_overflow", s.trace_overflow)
        .done();
  }
  std::string operator()(const trace::IoSnapshot& s) const {
    const socketcan_io::IoCounts& c = s.io;
    const auto rej = [&c](socketcan_io::RxClass k) {
      return c.rx_rejected[static_cast<std::size_t>(k)];
    };
    using socketcan_io::RxClass;
    return Json("io", s.at)
        .num("rx", c.rx)
        .raw("rx_rejected",
             object(P{"rtr", rej(RxClass::kRtr)}, P{"error", rej(RxClass::kError)},
                    P{"fd", rej(RxClass::kFd)}, P{"extended_id", rej(RxClass::kExtendedId)},
                    P{"bad_length", rej(RxClass::kBadLength)}))
        .num("rx_error", c.rx_error)
        .num("rx_reject_budget_hits", c.rx_reject_budget_hits)
        .num("tx", c.tx)
        .num("tx_eagain", c.tx_eagain)
        .num("tx_enobufs", c.tx_enobufs)
        .num("tx_other_error", c.tx_other_error)
        .num("trace_overflow", s.trace_overflow)
        .done();
  }
  std::string operator()(const trace::InjectIo& r) const {
    const char* kind = "recv_error";
    if (r.kind == trace::InjectIo::Kind::kSendError) {
      kind = "send_error";
    } else if (r.kind == trace::InjectIo::Kind::kTruncated) {
      kind = "truncated";
    }
    return Json("inject_io", r.at)
        .str("kind", kind)
        .num("value", r.value)
        .raw("inject", inject_json(r.totals))
        .done();
  }
  std::string operator()(const trace::Exit& e) const {
    return Json("exit", e.at)
        .num("trace_overflow", e.trace_overflow)
        .num("signal", e.signal)
        .raw("inject", inject_json(e.inject))
        .done();
  }
  static std::string inject_json(const trace::InjectCounts& c) {
    return object(P{"recv_errors", c.recv_errors}, P{"send_errors", c.send_errors},
                  P{"truncated", c.truncated});
  }
};

}  // namespace

std::string to_json(const TraceRecord& record) { return std::visit(ToJson{}, record); }

TraceRing::TraceRing() : slots_(std::make_unique<TraceRecord[]>(kCapacity)) {}

bool TraceRing::push(const TraceRecord& record) {
  const std::uint64_t head = head_.load(std::memory_order_relaxed);
  if (head - tail_.load(std::memory_order_acquire) >= kCapacity) {
    overflow_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  slots_[head % kCapacity] = record;
  head_.store(head + 1, std::memory_order_release);
  return true;
}

std::optional<TraceRecord> TraceRing::pop() {
  const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) {
    return std::nullopt;
  }
  TraceRecord record = slots_[tail % kCapacity];
  tail_.store(tail + 1, std::memory_order_release);
  return record;
}

TraceWriter::TraceWriter(TraceRing& ring, Sink sink, Flush flush, std::chrono::microseconds idle)
    : ring_(ring), sink_(std::move(sink)), flush_(std::move(flush)), idle_(idle) {}

TraceWriter::~TraceWriter() { stop(); }

void TraceWriter::start() {
  if (!thread_.joinable()) {
    stop_.store(false);
    thread_ = std::thread([this] { run(); });
  }
}

void TraceWriter::stop() {
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
  // Whatever arrived after the thread's last drain, or everything if it never ran.
  if (drain() && flush_) {
    flush_();
  }
}

bool TraceWriter::drain() {
  bool any = false;
  while (std::optional<TraceRecord> r = ring_.pop()) {
    sink_(*r);
    written_.fetch_add(1, std::memory_order_relaxed);
    any = true;
  }
  return any;
}

void TraceWriter::run() {
  // Polling with a short sleep: the producer never makes a system call to wake the writer.
  while (!stop_.load()) {
    if (drain()) {
      if (flush_) {
        flush_();
      }
    } else {
      std::this_thread::sleep_for(idle_);
    }
  }
  if (drain() && flush_) {
    flush_();
  }
}

}  // namespace vdrive_bank
