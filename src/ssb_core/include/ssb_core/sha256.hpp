#pragma once
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace ssb {

std::string Sha256Hex(const void* data, size_t size);
std::string Sha256File(const std::filesystem::path& path);

// Incremental hash for files written in pieces.
class Sha256Stream {
 public:
  Sha256Stream();
  ~Sha256Stream();
  Sha256Stream(const Sha256Stream&) = delete;
  Sha256Stream& operator=(const Sha256Stream&) = delete;
  void Update(const void* data, size_t size);
  std::string Final();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ssb
