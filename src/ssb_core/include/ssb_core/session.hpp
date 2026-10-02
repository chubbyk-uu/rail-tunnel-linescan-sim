#pragma once
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ssb_core/records.hpp"
#include "ssb_core/sha256.hpp"

namespace ssb {

nlohmann::json DtypeJson(const Fields& fields);
void WriteJsonAtomic(const std::filesystem::path& path, const nlohmann::json& value);
void WriteTextAtomic(const std::filesystem::path& path, const std::string& text);
void SyncFile(const std::filesystem::path& path);
void SyncDirectory(const std::filesystem::path& path);

struct IoStatistics {
  size_t files = 0, bytes = 0;
  double write_seconds = 0, readback_seconds = 0, sync_seconds = 0;
  double longest_write_s = 0, longest_sync_s = 0;
  void Merge(const IoStatistics& other);
  nlohmann::json Json() const;
};

// Append-only table of fixed-size records; Close() returns its manifest entry.
class TableWriter {
 public:
  TableWriter(std::filesystem::path path, nlohmann::json dtype, size_t record_size,
              std::function<void()> progress = {});
  void Append(const void* records, size_t count);
  template <class T>
  void Append(const std::vector<T>& records) {
    if (sizeof(T) != record_size_) throw std::logic_error("record size mismatch for " + path_.string());
    Append(records.data(), records.size());
  }
  nlohmann::json Close();
  const IoStatistics& Statistics() const { return statistics_; }

 private:
  std::filesystem::path path_;
  nlohmann::json dtype_;
  size_t record_size_, count_ = 0;
  std::ofstream out_;
  Sha256Stream hash_;
  bool closed_ = false;
  IoStatistics statistics_;
  std::function<void()> progress_;
};

// Raw pixel rows in fixed-size blocks (the tail block may be shorter). Each block is
// written to a temporary file, read back and hash-checked, then renamed into place.
class BlockWriter {
 public:
  BlockWriter(std::filesystem::path directory, int width, int block_rows,
              std::function<void()> progress = {});
  void Append(const uint8_t* rows, size_t count, int64_t first_sequence);
  nlohmann::json Close();
  const IoStatistics& Statistics() const { return statistics_; }
  int64_t RowsWritten() const { return next_sequence_; }
  // Flushed and hash-verified rows; durable only after Close's batch sync.
  int64_t RowsPersisted() const { return next_sequence_ - static_cast<int64_t>(buffered_); }

 private:
  void Flush();
  std::filesystem::path dir_;
  int width_, block_rows_;
  std::vector<uint8_t> buffer_;
  size_t buffered_ = 0;
  int64_t next_sequence_ = 0, block_first_ = 0;
  nlohmann::json blocks_ = nlohmann::json::array();
  bool closed_ = false;
  IoStatistics statistics_;
  std::function<void()> progress_;
};

// Build-time and run-time identity of the code producing a session (DESIGN.md §12.1).
nlohmann::json ProvenanceJson(const std::vector<std::string>& argv);

}  // namespace ssb
