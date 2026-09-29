#include "ssb_core/session.hpp"

#include <dlfcn.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "ssb_core/build_stamp.hpp"

namespace ssb {
namespace {

std::string RunCommand(const std::string& command) {
  std::array<char, 4096> buffer;
  std::string out;
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) return "";
  while (size_t n = fread(buffer.data(), 1, buffer.size(), pipe)) out.append(buffer.data(), n);
  pclose(pipe);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
  return out;
}

std::string ThisLibraryPath() {
  Dl_info info{};
  if (dladdr(reinterpret_cast<void*>(&ProvenanceJson), &info) && info.dli_fname) return info.dli_fname;
  return "";
}

}  // namespace

nlohmann::json DtypeJson(const Fields& fields) {
  nlohmann::json j = nlohmann::json::array();
  for (const auto& [name, type] : fields) j.push_back({name, type});
  return j;
}

void WriteTextAtomic(const std::filesystem::path& path, const std::string& text) {
  const auto tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out << text;
    out.flush();
    if (!out) throw std::runtime_error("write failed: " + tmp);
  }
  std::filesystem::rename(tmp, path);
}

void WriteJsonAtomic(const std::filesystem::path& path, const nlohmann::json& value) {
  WriteTextAtomic(path, value.dump(2) + "\n");
}

TableWriter::TableWriter(std::filesystem::path path, nlohmann::json dtype, size_t record_size)
    : path_(std::move(path)), dtype_(std::move(dtype)), record_size_(record_size) {
  out_.open(path_, std::ios::binary | std::ios::trunc);
  if (!out_) throw std::runtime_error("cannot create " + path_.string());
}

void TableWriter::Append(const void* records, size_t count) {
  if (closed_) throw std::logic_error("append to closed table " + path_.string());
  const size_t bytes = count * record_size_;
  out_.write(static_cast<const char*>(records), static_cast<std::streamsize>(bytes));
  if (!out_) throw std::runtime_error("write failed: " + path_.string());
  hash_.Update(records, bytes);
  count_ += count;
}

nlohmann::json TableWriter::Close() {
  if (closed_) throw std::logic_error("table closed twice");
  closed_ = true;
  out_.flush();
  out_.close();
  if (out_.fail()) throw std::runtime_error("close failed: " + path_.string());
  const std::string digest = hash_.Final();
  if (Sha256File(path_) != digest) throw std::runtime_error("read-back hash mismatch: " + path_.string());
  return {{"file", path_.filename().string()}, {"dtype", dtype_}, {"record_size", record_size_},
          {"count", count_}, {"sha256", digest}};
}

BlockWriter::BlockWriter(std::filesystem::path directory, int width, int block_rows)
    : dir_(std::move(directory)), width_(width), block_rows_(block_rows),
      buffer_(static_cast<size_t>(width) * block_rows) {}

void BlockWriter::Append(const uint8_t* rows, size_t count, int64_t first_sequence) {
  if (closed_) throw std::logic_error("append to closed block writer");
  if (first_sequence != next_sequence_)
    throw std::runtime_error("raw rows out of order: expected sequence " + std::to_string(next_sequence_) +
                             ", got " + std::to_string(first_sequence));
  for (size_t i = 0; i < count; ++i) {
    std::copy(rows + i * width_, rows + (i + 1) * width_, buffer_.begin() + buffered_ * width_);
    ++buffered_;
    ++next_sequence_;
    if (buffered_ == static_cast<size_t>(block_rows_)) Flush();
  }
}

void BlockWriter::Flush() {
  if (buffered_ == 0) return;
  std::ostringstream name;
  name << "block_" << std::setw(6) << std::setfill('0') << blocks_.size() << ".u8";
  const auto path = dir_ / name.str();
  const size_t bytes = buffered_ * width_;
  const std::string digest = Sha256Hex(buffer_.data(), bytes);
  const auto tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(buffer_.data()), static_cast<std::streamsize>(bytes));
    out.close();
    if (out.fail()) throw std::runtime_error("block write failed: " + tmp);
  }
  if (Sha256File(tmp) != digest) throw std::runtime_error("block read-back hash mismatch: " + tmp);
  std::filesystem::rename(tmp, path);
  blocks_.push_back({{"file", name.str()}, {"first_sequence", block_first_}, {"rows", buffered_},
                     {"sha256", digest}});
  block_first_ += static_cast<int64_t>(buffered_);
  buffered_ = 0;
}

nlohmann::json BlockWriter::Close() {
  if (closed_) throw std::logic_error("block writer closed twice");
  Flush();  // the tail block is kept, however short
  closed_ = true;
  return {{"schema", "ssb.raw_blocks.v1"}, {"width", width_}, {"block_rows", block_rows_},
          {"pixel_format", "mono8"}, {"rows", next_sequence_}, {"blocks", blocks_}};
}

nlohmann::json ProvenanceJson(const std::vector<std::string>& argv) {
  const std::string source = SSB_SOURCE_DIR;
  std::istringstream state(RunCommand("sh '" + source + "/cmake/source_state.sh' '" + source + "' 2>/dev/null"));
  std::string head = "unknown", digest = "unknown";
  int dirty = 1;
  state >> head >> dirty >> digest;
  std::error_code ec;
  const auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  const std::string lib = ThisLibraryPath();
  const nlohmann::json build = {{"git_head", SSB_BUILD_GIT_HEAD}, {"git_dirty", SSB_BUILD_GIT_DIRTY != 0},
                                {"source_digest", SSB_BUILD_GIT_DIFF_SHA256}, {"build_type", SSB_BUILD_TYPE}};
  const nlohmann::json run = {{"git_head", head}, {"git_dirty", dirty != 0}, {"source_digest", digest}};
  // False means the running binary was built from a different source tree than the
  // one on disk now: rebuild before treating the session as evidence.
  const bool matches = head == SSB_BUILD_GIT_HEAD && digest == SSB_BUILD_GIT_DIFF_SHA256;
  return {{"schema", "ssb.provenance.v1"},
          {"argv", argv},
          {"source_dir", source},
          {"build", build},
          {"source_at_run", run},
          {"binary_matches_source", matches},
          {"executable", exe.string()},
          {"executable_sha256", exe.empty() ? "" : Sha256File(exe)},
          {"library", lib},
          {"library_sha256", lib.empty() || lib == exe.string() ? "" : Sha256File(lib)}};
}

}  // namespace ssb
