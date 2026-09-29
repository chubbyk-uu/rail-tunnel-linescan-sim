#include "ssb_core/sha256.hpp"

#include <openssl/evp.h>

#include <fstream>
#include <stdexcept>
#include <vector>

namespace ssb {

struct Sha256Stream::Impl {
  EVP_MD_CTX* ctx = nullptr;
  bool done = false;
};

Sha256Stream::Sha256Stream() : impl_(std::make_unique<Impl>()) {
  impl_->ctx = EVP_MD_CTX_new();
  if (!impl_->ctx || EVP_DigestInit_ex(impl_->ctx, EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA-256 init failed");
}

Sha256Stream::~Sha256Stream() { EVP_MD_CTX_free(impl_->ctx); }

void Sha256Stream::Update(const void* data, size_t size) {
  if (impl_->done) throw std::logic_error("SHA-256 update after final");
  if (size && EVP_DigestUpdate(impl_->ctx, data, size) != 1) throw std::runtime_error("SHA-256 update failed");
}

std::string Sha256Stream::Final() {
  if (impl_->done) throw std::logic_error("SHA-256 final twice");
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(impl_->ctx, digest, &length) != 1) throw std::runtime_error("SHA-256 final failed");
  impl_->done = true;
  static const char* hex = "0123456789abcdef";
  std::string out;
  for (unsigned i = 0; i < length; ++i) {
    out.push_back(hex[digest[i] >> 4]);
    out.push_back(hex[digest[i] & 15]);
  }
  return out;
}

std::string Sha256Hex(const void* data, size_t size) {
  Sha256Stream s;
  s.Update(data, size);
  return s.Final();
}

std::string Sha256File(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open for hashing: " + path.string());
  Sha256Stream s;
  std::vector<char> buffer(1 << 20);
  while (in) {
    in.read(buffer.data(), buffer.size());
    s.Update(buffer.data(), static_cast<size_t>(in.gcount()));
  }
  if (in.bad()) throw std::runtime_error("read failed while hashing: " + path.string());
  return s.Final();
}

}  // namespace ssb
