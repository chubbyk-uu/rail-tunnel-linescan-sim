// Public-image reconstruction only. No renderer, scene or truth dependencies.
#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
constexpr size_t kBudget = 256ull << 20;
constexpr int kMaxPixels = 1 << 20;
thread_local std::string last_error;

void checked(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
template<class Function> int guarded(Function function) {
  try { function(); last_error.clear(); return 0; }
  catch (const std::exception& error) { last_error = error.what(); return -1; }
}

struct Row {
  int lower, upper;
  double weight;
  int supported, reserved;
};
static_assert(sizeof(Row) == 24);

struct Buffer {
  void* pointer = nullptr;
  size_t capacity = 0;
  ~Buffer() { if (pointer) cudaFree(pointer); }
  void grow(size_t size) {
    if (size <= capacity) return;
    if (pointer) checked(cudaFree(pointer));
    pointer = nullptr; capacity = 0;
    checked(cudaMalloc(&pointer, size)); capacity = size;
  }
  template<class T> T* as() { return static_cast<T*>(pointer); }
  void upload(const void* data, size_t size) {
    grow(size); checked(cudaMemcpy(pointer, data, size, cudaMemcpyHostToDevice));
  }
};

struct Context {
  int width, rows = 0, columns = 0;
  std::string device;
  size_t peak = 0;
  Buffer native_offsets, output_offsets, geometry_valid;
  Buffer pixels, axes, row_sources, xs;
  Buffer image, count, source, best;
  void reserve_begin(size_t n, size_t columns) {
    size_t total = native_offsets.capacity+output_offsets.capacity+geometry_valid.capacity+
                   pixels.capacity+axes.capacity+row_sources.capacity;
    total += std::max(xs.capacity, columns*sizeof(double))+
             std::max(image.capacity, n*sizeof(float))+
             std::max(count.capacity, n*sizeof(uint16_t))+
             std::max(source.capacity, n*sizeof(int16_t))+
             std::max(best.capacity, n*sizeof(float));
    if (total > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
  }
  void reserve_check(size_t pixels_bytes, size_t axes_bytes, size_t row_bytes) {
    size_t total = native_offsets.capacity+output_offsets.capacity+geometry_valid.capacity+
        xs.capacity+image.capacity+count.capacity+source.capacity+best.capacity;
    total += std::max(pixels.capacity, pixels_bytes)+std::max(axes.capacity, axes_bytes)+
             std::max(row_sources.capacity, row_bytes);
    if (total > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
  }
  void account() {
    size_t total = 0;
    for (auto* b : {&native_offsets, &output_offsets, &geometry_valid, &pixels,
                    &axes, &row_sources, &xs, &image, &count, &source, &best})
      total += b->capacity;
    peak = std::max(peak, total);
    if (total > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
  }
};

__device__ double inverse_column(double x, const double* offsets, int width) {
  if (x <= offsets[0]) return 0.;
  if (x >= offsets[width-1]) return width-1.;
  int left = 0, right = width-1;
  while (right-left > 1) {
    const int mid = (left+right)/2;
    if (offsets[mid] <= x) left = mid; else right = mid;
  }
  // Same linear inverse convention as np.interp; no approximate lookup table.
  const double slope = 1./(offsets[right]-offsets[left]);
  return slope*(x-offsets[left])+left;
}

struct Sample { float value; double column; bool valid; };
__device__ Sample along(int row, double x, const float* pixels, const double* axes,
                        const double* native, const double* output,
                        const uint8_t* geometry, int width) {
  const double delta = x-axes[row];
  const double column = inverse_column(delta, native, width);
  const int left = static_cast<int>(floor(column));
  const int right = min(left+1, width-1);
  const float weight = static_cast<float>(column-left);
  const float a = pixels[static_cast<size_t>(row)*width+left];
  const float b = pixels[static_cast<size_t>(row)*width+right];
  const double corrected = inverse_column(delta, output, width);
  const int c0 = static_cast<int>(floor(corrected));
  const int c1 = min(c0+1, width-1);
  const bool valid = delta >= native[0] && delta <= native[width-1] &&
      delta >= output[0] && delta <= output[width-1] &&
      geometry[c0] && geometry[c1] && isfinite(a) && isfinite(b);
  return {a*(1.f-weight)+b*weight, column, valid};
}

__global__ void initialize(float* image, uint16_t* count, int16_t* source,
                           float* best, int pixels) {
  const int i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i >= pixels) return;
  image[i] = nanf(""); count[i] = 0; source[i] = -1; best[i] = -INFINITY;
}

__global__ void sample_band(const float* pixels, const double* axes, const Row* rows,
    const double* xs, const double* native, const double* output, const uint8_t* geometry,
    int width, int nq, int nx, int left, int right, int band,
    float* image, uint16_t* count, int16_t* source, float* best) {
  const int i = blockIdx.x*blockDim.x+threadIdx.x;
  const int span = right-left;
  if (i >= nq*span) return;
  const int q = i/span, column = left+i%span;
  const Row row = rows[q];
  if (!row.supported) return;
  const Sample a = along(row.lower, xs[column], pixels, axes, native, output, geometry, width);
  const Sample b = along(row.upper, xs[column], pixels, axes, native, output, geometry, width);
  if (!a.valid || !b.valid) return;
  const int target = q*nx+column;
  ++count[target];
  const double score = fmin(a.column, width-1.-a.column);
  if (score > best[target]) {
    image[target] = static_cast<float>(static_cast<double>(a.value)*(1.-row.weight)+
                                       static_cast<double>(b.value)*row.weight);
    best[target] = static_cast<float>(score);
    source[target] = static_cast<int16_t>(band);
  }
}
}  // namespace

extern "C" {
int ssb_unroll_abi() { return 1; }
const char* ssb_unroll_error() { return last_error.c_str(); }

void* ssb_unroll_create(int width, const double* native, const double* output,
                        const uint8_t* geometry) {
  Context* result = nullptr;
  guarded([&] {
    if (width < 2 || width > 65536) throw std::runtime_error("invalid sensor width");
    auto context = std::make_unique<Context>(); context->width = width;
    int device; checked(cudaGetDevice(&device));
    cudaDeviceProp properties{}; checked(cudaGetDeviceProperties(&properties, device));
    context->device = properties.name;
    context->native_offsets.upload(native, width*sizeof(double));
    context->output_offsets.upload(output, width*sizeof(double));
    context->geometry_valid.upload(geometry, width);
    context->account(); result = context.release();
  });
  return result;
}
void ssb_unroll_destroy(void* handle) { delete static_cast<Context*>(handle); }
const char* ssb_unroll_device(void* handle) { return static_cast<Context*>(handle)->device.c_str(); }
size_t ssb_unroll_peak_bytes(void* handle) { return static_cast<Context*>(handle)->peak; }

int ssb_unroll_begin(void* handle, int rows, int columns, const double* xs) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    if (rows <= 0 || columns <= 0 || static_cast<int64_t>(rows)*columns > kMaxPixels)
      throw std::runtime_error("invalid CUDA tile dimensions");
    c.rows = rows; c.columns = columns;
    const size_t n = static_cast<size_t>(rows)*columns;
    c.reserve_begin(n, columns);
    c.image.grow(n*sizeof(float)); c.count.grow(n*sizeof(uint16_t));
    c.source.grow(n*sizeof(int16_t)); c.best.grow(n*sizeof(float));
    c.xs.upload(xs, columns*sizeof(double)); c.account();
    initialize<<<(n+255)/256,256>>>(c.image.as<float>(), c.count.as<uint16_t>(),
        c.source.as<int16_t>(), c.best.as<float>(), n);
    checked(cudaGetLastError());
  });
}
int ssb_unroll_band(void* handle, int native_rows, const float* pixels,
    const double* axes, const void* row_data, int left, int right, int band) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    const auto* rows = static_cast<const Row*>(row_data);
    if (native_rows <= 0 || left < 0 || right > c.columns || right <= left ||
        band < 0 || band >= std::numeric_limits<int16_t>::max())
      throw std::runtime_error("invalid CUDA band dimensions");
    const size_t bytes = static_cast<size_t>(native_rows)*c.width*sizeof(float);
    if (bytes > kBudget/2) throw std::runtime_error("CUDA native input exceeds 128 MiB");
    c.reserve_check(bytes, native_rows*sizeof(double), c.rows*sizeof(Row));
    for (int i = 0; i < native_rows; ++i)
      if (!std::isfinite(axes[i])) throw std::runtime_error("nonfinite CUDA axis position");
    for (int q = 0; q < c.rows; ++q)
      if (rows[q].lower < 0 || rows[q].upper < 0 || rows[q].lower >= native_rows ||
          rows[q].upper >= native_rows || !std::isfinite(rows[q].weight))
        throw std::runtime_error("invalid CUDA source row");
    c.pixels.upload(pixels, bytes); c.axes.upload(axes, native_rows*sizeof(double));
    c.row_sources.upload(rows, c.rows*sizeof(Row)); c.account();
    const int n = c.rows*(right-left);
    sample_band<<<(n+255)/256,256>>>(c.pixels.as<float>(), c.axes.as<double>(), c.row_sources.as<Row>(),
        c.xs.as<double>(), c.native_offsets.as<double>(), c.output_offsets.as<double>(),
        c.geometry_valid.as<uint8_t>(), c.width, c.rows, c.columns, left, right, band,
        c.image.as<float>(), c.count.as<uint16_t>(), c.source.as<int16_t>(), c.best.as<float>());
    checked(cudaGetLastError());
  });
}
int ssb_unroll_finish(void* handle, float* image, uint16_t* count, int16_t* source) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    checked(cudaDeviceSynchronize());
    const size_t n = static_cast<size_t>(c.rows)*c.columns;
    checked(cudaMemcpy(image, c.image.pointer, n*sizeof(float), cudaMemcpyDeviceToHost));
    checked(cudaMemcpy(count, c.count.pointer, n*sizeof(uint16_t), cudaMemcpyDeviceToHost));
    checked(cudaMemcpy(source, c.source.pointer, n*sizeof(int16_t), cudaMemcpyDeviceToHost));
  });
}
}  // extern "C"
