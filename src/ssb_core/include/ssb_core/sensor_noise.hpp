#pragma once
#include <cmath>
#include <cstdint>

#ifdef __CUDACC__
#define SSB_NOISE_HD __host__ __device__
#else
#define SSB_NOISE_HD
#endif

namespace ssb {
// Generator-only parameters. Neither seeds nor true response are public inputs.
struct SensorNoise {
  bool enabled;
  uint64_t pattern_seed, realization_seed;
  float electrons_per_dn, read_noise_e;
  float dark_current_e_per_s, bias_dn, prnu_fraction;
};

inline SensorNoise DefaultSensorNoise() { return {false,20261003,1,20.f,8.f,100.f,4.f,.005f}; }

SSB_NOISE_HD inline uint64_t NoiseHash(uint64_t value) {
  value += 0x9e3779b97f4a7c15ull;
  value = (value^(value>>30))*0xbf58476d1ce4e5b9ull;
  value = (value^(value>>27))*0x94d049bb133111ebull;
  return value^(value>>31);
}

SSB_NOISE_HD inline float NoiseUniform(uint64_t key, uint64_t stream) {
  // Open interval, including on float32: 23 random bits and a half-bin offset.
  return (static_cast<float>(NoiseHash(key^NoiseHash(stream))>>41)+.5f)/8388608.f;
}

SSB_NOISE_HD inline float NoiseNormal(uint64_t key, uint64_t stream) {
  return sqrtf(-2.f*logf(NoiseUniform(key, 2*stream)))*
         cosf(6.283185307179586f*NoiseUniform(key, 2*stream+1));
}

SSB_NOISE_HD inline float PhotonCount(float mean, uint64_t key) {
  if (mean <= 0.f) return 0.f;
  if (mean >= 64.f) {
    // Declared high-count Gaussian approximation, rounded to integral electrons.
    // This is not advertised as exact Poisson in the high-count regime.
    return fmaxf(0.f, floorf(mean+sqrtf(mean)*NoiseNormal(key, 1)+.5f));
  }
  // Exact inverse Poisson CDF in the low-count regime. One uniform, no unbounded
  // rejection loop or per-pixel state. The 512-count tail is < float resolution.
  const double uniform = NoiseUniform(key, 4);
  double probability = exp(-static_cast<double>(mean)), sum = probability;
  for (int count = 0; count < 512; ++count) {
    if (uniform <= sum) return static_cast<float>(count);
    probability *= mean/(count+1.); sum += probability;
  }
  return 512.f;
}

SSB_NOISE_HD inline float SensorDN(float ideal_dn, float column_response,
                                  uint64_t row_sequence, unsigned column,
                                  const SensorNoise& noise, double exposure_s) {
  if (!noise.enabled) return ideal_dn;
  const uint64_t key = NoiseHash(noise.realization_seed+0xbb67ae8584caa73bull)^NoiseHash(row_sequence+0x3c6ef372fe94f82bull)^NoiseHash(uint64_t(column)+0x6a09e667f3bcc909ull);
  const float mean = fmaxf(0.f, ideal_dn)*column_response*noise.electrons_per_dn+
                     static_cast<float>(exposure_s)*noise.dark_current_e_per_s;
  return noise.bias_dn+(PhotonCount(mean, key)+noise.read_noise_e*NoiseNormal(key, 5))/noise.electrons_per_dn;
}

inline float ColumnResponse(const SensorNoise& noise, unsigned column) {
  // Positive lognormal fixed column sensitivity with expectation one.
  const float sigma = sqrtf(log1pf(noise.prnu_fraction*noise.prnu_fraction));
  return expf(sigma*NoiseNormal(NoiseHash(noise.pattern_seed)^NoiseHash(column), 9)-.5f*sigma*sigma);
}
}  // namespace ssb
#undef SSB_NOISE_HD
