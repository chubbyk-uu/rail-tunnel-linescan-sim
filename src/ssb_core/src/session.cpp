#include "ssb_core/session.hpp"

#include <cstdlib>

#include <dlfcn.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <chrono>
#include <system_error>

#include <array>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "ssb_core/build_stamp.hpp"

namespace ssb {
namespace {
using Clock = std::chrono::steady_clock;
double Elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now()-start).count();
}

void SyncPath(const std::filesystem::path& path, bool directory) {
  const int fd = ::open(path.c_str(), O_RDONLY|O_CLOEXEC|(directory ? O_DIRECTORY : 0));
  if(fd<0) throw std::system_error(errno,std::generic_category(),"open for fsync: "+path.string());
  int result;
  do { result=::fsync(fd); } while(result<0 && errno==EINTR);
  const int error=errno;
  const int closed=::close(fd);
  if(result<0) throw std::system_error(error,std::generic_category(),"fsync: "+path.string());
  if(closed<0) throw std::system_error(errno,std::generic_category(),"close after fsync: "+path.string());
}

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

void SyncFile(const std::filesystem::path& path) { SyncPath(path,false); }
void SyncDirectory(const std::filesystem::path& path) { SyncPath(path,true); }

void IoStatistics::Merge(const IoStatistics& other) {
  files+=other.files;bytes+=other.bytes;
  write_seconds+=other.write_seconds;readback_seconds+=other.readback_seconds;
  sync_seconds+=other.sync_seconds;
  longest_write_s=std::max(longest_write_s,other.longest_write_s);
  longest_sync_s=std::max(longest_sync_s,other.longest_sync_s);
}

nlohmann::json IoStatistics::Json() const {
  return {{"files",files},{"bytes",bytes},{"write_seconds",write_seconds},
          {"readback_seconds",readback_seconds},{"sync_seconds",sync_seconds},
          {"longest_write_s",longest_write_s},{"longest_sync_s",longest_sync_s}};
}

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
    out.close();
    if (out.fail()) throw std::runtime_error("write failed: " + tmp);
  }
  SyncFile(tmp);
  std::filesystem::rename(tmp, path);
  SyncDirectory(path.parent_path().empty() ? "." : path.parent_path());
}

void WriteJsonAtomic(const std::filesystem::path& path, const nlohmann::json& value) {
  WriteTextAtomic(path, value.dump(2) + "\n");
}

TableWriter::TableWriter(std::filesystem::path path, nlohmann::json dtype, size_t record_size,
                         std::function<void()> progress)
    : path_(std::move(path)), dtype_(std::move(dtype)), record_size_(record_size),
      progress_(std::move(progress)) {
  out_.open(path_, std::ios::binary | std::ios::trunc);
  if (!out_) throw std::runtime_error("cannot create " + path_.string());
}

void TableWriter::Append(const void* records, size_t count) {
  if (closed_) throw std::logic_error("append to closed table " + path_.string());
  const size_t bytes = count * record_size_;
  const auto begin=Clock::now();
  out_.write(static_cast<const char*>(records), static_cast<std::streamsize>(bytes));
  if (!out_) throw std::runtime_error("write failed: " + path_.string());
  const double elapsed=Elapsed(begin);
  hash_.Update(records, bytes);
  count_ += count;
  statistics_.write_seconds+=elapsed;
  statistics_.longest_write_s=std::max(statistics_.longest_write_s,elapsed);
  statistics_.bytes+=bytes;
}

nlohmann::json TableWriter::Close() {
  if (closed_) throw std::logic_error("table closed twice");
  closed_ = true;
  const auto flush_begin=Clock::now();
  out_.flush();
  out_.close();
  if (out_.fail()) throw std::runtime_error("close failed: " + path_.string());
  if (progress_) progress_();
  const double flush_seconds=Elapsed(flush_begin);
  statistics_.write_seconds+=flush_seconds;
  statistics_.longest_write_s=std::max(statistics_.longest_write_s,flush_seconds);
  const std::string digest = hash_.Final();
  auto begin=Clock::now();
  if (Sha256File(path_) != digest) throw std::runtime_error("read-back hash mismatch: " + path_.string());
  if (progress_) progress_();
  statistics_.readback_seconds=Elapsed(begin);
  begin=Clock::now();SyncFile(path_);
  if (progress_) progress_();
  statistics_.sync_seconds=statistics_.longest_sync_s=Elapsed(begin);
  statistics_.files=1;
  return {{"file", path_.filename().string()}, {"dtype", dtype_}, {"record_size", record_size_},
          {"count", count_}, {"sha256", digest}};
}

BlockWriter::BlockWriter(std::filesystem::path directory, int width, int block_rows,
                         std::function<void()> progress)
    : dir_(std::move(directory)), width_(width), block_rows_(block_rows),
      buffer_(static_cast<size_t>(width) * block_rows), progress_(std::move(progress)) {}

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
  const auto begin=Clock::now();
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(buffer_.data()), static_cast<std::streamsize>(bytes));
    out.close();
    if (out.fail()) throw std::runtime_error("block write failed: " + tmp);
  }
  const double write_s=Elapsed(begin);
  statistics_.write_seconds+=write_s;
  statistics_.longest_write_s=std::max(statistics_.longest_write_s,write_s);
  const auto read_begin=Clock::now();
  if (Sha256File(tmp) != digest) throw std::runtime_error("block read-back hash mismatch: " + tmp);
  statistics_.readback_seconds+=Elapsed(read_begin);
  statistics_.bytes+=bytes;++statistics_.files;
  std::filesystem::rename(tmp, path);
  blocks_.push_back({{"file", name.str()}, {"first_sequence", block_first_}, {"rows", buffered_},
                     {"sha256", digest}});
  block_first_ += static_cast<int64_t>(buffered_);
  buffered_ = 0;
}

nlohmann::json BlockWriter::Close() {
  if (closed_) throw std::logic_error("block writer closed twice");
  Flush();  // the tail block is kept, however short
  // Sync once per completed block at the task boundary, never per row/preview.
  for(const auto& block:blocks_) {
    const auto begin=Clock::now();SyncFile(dir_/block.at("file").get<std::string>());
    if (progress_) progress_();
    const double seconds=Elapsed(begin);
    statistics_.sync_seconds+=seconds;
    statistics_.longest_sync_s=std::max(statistics_.longest_sync_s,seconds);
  }
  const auto begin=Clock::now();SyncDirectory(dir_);
  if (progress_) progress_();
  statistics_.sync_seconds+=Elapsed(begin);
  closed_ = true;
  return {{"schema", "ssb.raw_blocks.v1"}, {"width", width_}, {"block_rows", block_rows_},
          {"pixel_format", "mono8"}, {"rows", next_sequence_}, {"blocks", blocks_}};
}

std::optional<double> ArchivedPlannedEnd(const std::filesystem::path& pose_stream,
                                         std::filesystem::path* session_file) {
  const auto evaluation = pose_stream.parent_path();
  const auto source = evaluation.parent_path() / "session.json", manifest_path = evaluation / "manifest.json";
  if (!std::filesystem::is_regular_file(source) || !std::filesystem::is_regular_file(manifest_path) ||
      !std::filesystem::is_regular_file(pose_stream)) return std::nullopt;
  nlohmann::json summary, manifest;
  std::ifstream(source) >> summary;
  std::ifstream(manifest_path) >> manifest;
  const auto files = summary.value("files", nlohmann::json::object());
  const auto motion = summary.value("motion", nlohmann::json::object());
  const auto entry = manifest.value("pose_stream", nlohmann::json::object());
  if (summary.value("status", "") != "complete" || motion.value("complete", nlohmann::json()) != true ||
      !motion.contains("planned_end_s") || !motion.at("planned_end_s").is_number() ||
      files.value("evaluation/manifest.json", "") != Sha256File(manifest_path) ||
      entry.value("file", "") != pose_stream.filename().string() ||
      entry.value("sha256", "") != Sha256File(pose_stream))
    return std::nullopt;
  if (session_file) *session_file = source;
  return motion.at("planned_end_s").get<double>();
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
  // Set by tools/ssb_runtime.sh and the WSL OptiX wrapper; null when launched without them.
  auto environment = [](const char* name) -> nlohmann::json {
    const char* value = std::getenv(name);
    return value ? nlohmann::json(value) : nlohmann::json();
  };
  const nlohmann::json runtime = {{"mode", environment("SSB_RUNTIME_MODE")},
                                  {"optix_runtime", environment("SSB_OPTIX_RUNTIME")},
                                  {"optix_driver_version", environment("SSB_OPTIX_DRIVER_VERSION")}};
  return {{"schema", "ssb.provenance.v1"},
          {"runtime", runtime},
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
