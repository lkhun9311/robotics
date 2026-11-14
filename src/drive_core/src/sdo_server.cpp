// SPDX-License-Identifier: Apache-2.0
#include "drive_core/sdo_server.hpp"

namespace drive_core {

namespace {

namespace cc = canopen_codec;
namespace od = canopen_codec::od;

bool readable(od::Access access) { return access != od::Access::kWo; }

bool writable(od::Access access) {
  return access == od::Access::kWo || access == od::Access::kRw || access == od::Access::kRwr ||
         access == od::Access::kRww;
}

// The value as the entry's type sees it: sign-extended from `size` bytes for a signed type.
std::int64_t typed_value(const od::EntryInfo& entry, std::uint32_t raw) {
  if (!entry.is_signed || entry.size == 0U || entry.size > 4U) {
    return static_cast<std::int64_t>(raw);
  }
  const unsigned bits = 8U * entry.size;
  const std::uint64_t sign = std::uint64_t{1} << (bits - 1U);
  const std::uint64_t v = raw;
  return static_cast<std::int64_t>((v ^ sign)) - static_cast<std::int64_t>(sign);
}

std::uint32_t low_bytes(std::uint32_t value, std::uint8_t size) {
  return size >= 4 ? value : value & ((std::uint32_t{1} << (8U * size)) - 1U);
}

cc::Sdo abort_response(od::Address address, std::uint32_t code) {
  cc::Sdo out{};
  out.type = cc::SdoType::kAbortToClient;
  out.address = address;
  out.abort_code = code;
  return out;
}

}  // namespace

const od::EntryInfo* SdoServer::find(od::Address address) {
  for (const od::EntryInfo& entry : od::kEntries) {
    if (entry.index == address.index && entry.sub == address.sub) {
      return &entry;
    }
  }
  return nullptr;
}

bool SdoServer::has_index(std::uint16_t index) {
  for (const od::EntryInfo& entry : od::kEntries) {
    if (entry.index == index) {
      return true;
    }
  }
  return false;
}

std::optional<cc::Sdo> SdoServer::handle(const cc::Sdo& request, ObjectAccess& objects) {
  if (request.type != cc::SdoType::kUploadRequest &&
      request.type != cc::SdoType::kDownloadRequest) {
    return std::nullopt;  // A client abort ends a transfer; there is nothing to answer.
  }
  const od::Address address = request.address;
  const od::EntryInfo* entry = find(address);
  if (entry == nullptr) {
    return abort_response(address,
                          has_index(address.index) ? sdo_abort::kNoSubIndex : sdo_abort::kNoObject);
  }
  if (request.type == cc::SdoType::kUploadRequest) {
    if (!readable(entry->access)) {
      return abort_response(address, sdo_abort::kWriteOnly);
    }
    const ObjectAccess::ReadResult r = objects.read(*entry);
    if (r.abort != 0U) {
      return abort_response(address, r.abort);
    }
    cc::Sdo out{};
    out.type = cc::SdoType::kUploadResponse;
    out.address = address;
    out.size = entry->size;
    out.value = low_bytes(r.value, entry->size);
    return out;
  }
  if (!writable(entry->access)) {
    return abort_response(address, sdo_abort::kReadOnly);
  }
  // Size 0: an expedited download with the size unspecified. The entry's size applies, and the
  // data bytes beyond it must be zero.
  const bool size_ok = request.size == 0U ? low_bytes(request.value, entry->size) == request.value
                                          : request.size == entry->size;
  if (!size_ok) {
    return abort_response(address, sdo_abort::kLength);
  }
  const std::int64_t value = typed_value(*entry, request.value);
  if ((entry->has_low && value < entry->low) || (entry->has_high && value > entry->high)) {
    return abort_response(address, sdo_abort::kRange);
  }
  if (const std::uint32_t code = objects.write(*entry, request.value); code != 0U) {
    return abort_response(address, code);
  }
  cc::Sdo out{};
  out.type = cc::SdoType::kDownloadResponse;
  out.address = address;
  return out;
}

}  // namespace drive_core
