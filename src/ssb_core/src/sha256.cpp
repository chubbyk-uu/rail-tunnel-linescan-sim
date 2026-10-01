#include "ssb_core/sha256.hpp"

#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>

#include <fstream>
#include <stdexcept>
#include <vector>

namespace ssb {

namespace {
std::string Hex(const unsigned char* bytes, size_t size) {
  const char* digits = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < size; ++i) {
    out.push_back(digits[bytes[i] >> 4]);
    out.push_back(digits[bytes[i] & 15]);
  }
  return out;
}
}  // namespace

std::string RandomKeyHex() {
  unsigned char bytes[32];
  if (RAND_bytes(bytes, sizeof(bytes)) != 1) throw std::runtime_error("optical key generation failed");
  return Hex(bytes, sizeof(bytes));
}

std::string HmacSha256Hex(const std::string& key_hex, const std::string& message) {
  if (key_hex.size() != 64 || key_hex.find_first_not_of("0123456789abcdef") != std::string::npos)
    throw std::invalid_argument("optical key must be 32 random bytes in lowercase hex");
  unsigned char key[32], digest[EVP_MAX_MD_SIZE];
  for (size_t i = 0; i < sizeof(key); ++i) key[i] = std::stoul(key_hex.substr(2*i, 2), nullptr, 16);
  unsigned length = 0;
  if (!HMAC(EVP_sha256(), key, sizeof(key), reinterpret_cast<const unsigned char*>(message.data()),
            message.size(), digest, &length)) throw std::runtime_error("optical HMAC failed");
  return Hex(digest, length);
}

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
