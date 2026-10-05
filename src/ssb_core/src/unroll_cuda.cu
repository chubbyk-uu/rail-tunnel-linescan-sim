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
  Buffer native_offsets, output_offsets, geometry_valid, flat_offset, flat_gain, flat_valid;
  Buffer pixels, axes, row_sources, xs;
  Buffer image, count, source, best;
  Buffer global_rays, global_qs, global_depth;
  Buffer runner_image, runner_score, runner_source;
  bool fusion = false;
  double fusion_gain = 1., feather_columns = 0.;
  size_t fusion_capacity() const {
    return runner_image.capacity+runner_score.capacity+runner_source.capacity;
  }
  bool use_depth = false;
  void reserve_begin(size_t n, size_t columns) {
    size_t total = fusion_capacity()+native_offsets.capacity+output_offsets.capacity+geometry_valid.capacity+
                   flat_offset.capacity+flat_gain.capacity+flat_valid.capacity+
                   pixels.capacity+axes.capacity+row_sources.capacity+global_rays.capacity+global_qs.capacity+global_depth.capacity;
    total += std::max(xs.capacity, columns*sizeof(double))+
             std::max(image.capacity, n*sizeof(float))+
             std::max(count.capacity, n*sizeof(uint16_t))+
             std::max(source.capacity, n*sizeof(int16_t))+
             std::max(best.capacity, n*sizeof(float));
    if (total > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
  }
  void reserve_check(size_t pixels_bytes, size_t axes_bytes, size_t row_bytes) {
    size_t total = fusion_capacity()+native_offsets.capacity+output_offsets.capacity+geometry_valid.capacity+
        flat_offset.capacity+flat_gain.capacity+flat_valid.capacity+
        xs.capacity+image.capacity+count.capacity+source.capacity+best.capacity+
        global_rays.capacity+global_qs.capacity+global_depth.capacity;
    total += std::max(pixels.capacity, pixels_bytes)+std::max(axes.capacity, axes_bytes)+
             std::max(row_sources.capacity, row_bytes);
    if (total > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
  }
  void account() {
    size_t total = 0;
    for (auto* b : {&native_offsets, &output_offsets, &geometry_valid, &flat_offset, &flat_gain, &flat_valid, &pixels,
                    &axes, &row_sources, &xs, &image, &count, &source, &best, &global_rays, &global_qs, &global_depth, &runner_image, &runner_score, &runner_source})
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

struct Flat { const float* offset; const float* gain; const uint8_t* valid; };
// Same float32 formula as ssb_tools.native_rows.correct; fmad is disabled for this library.
__device__ float corrected(uint8_t raw, int column, const Flat& flat, bool* ok) {
  *ok = flat.valid[column] && raw != 255;
  return (static_cast<float>(raw)-flat.offset[column])*flat.gain[column];
}

struct Sample { float value; double column; bool valid; };
__device__ Sample along(int row, double x, const uint8_t* pixels, const double* axes,
                        const double* native, const double* output,
                        const uint8_t* geometry, const Flat& flat, int width) {
  const double delta = x-axes[row];
  const double column = inverse_column(delta, native, width);
  const int left = static_cast<int>(floor(column));
  const int right = min(left+1, width-1);
  const float weight = static_cast<float>(column-left);
  bool ok_a, ok_b;
  const float a = corrected(pixels[static_cast<size_t>(row)*width+left], left, flat, &ok_a);
  const float b = corrected(pixels[static_cast<size_t>(row)*width+right], right, flat, &ok_b);
  const double corrected = inverse_column(delta, output, width);
  const int c0 = static_cast<int>(floor(corrected));
  const int c1 = min(c0+1, width-1);
  const bool valid = delta >= native[0] && delta <= native[width-1] &&
      delta >= output[0] && delta <= output[width-1] &&
      geometry[c0] && geometry[c1] && ok_a && ok_b;
  return {a*(1.f-weight)+b*weight, column, valid};
}

__global__ void initialize(float* image, uint16_t* count, int16_t* source,
                           float* best, int pixels) {
  const int i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i >= pixels) return;
  image[i] = nanf(""); count[i] = 0; source[i] = -1; best[i] = -INFINITY;
}

__global__ void initialize_runner(float* image, float* score, int16_t* source, int pixels) {
  int i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i >= pixels) return;
  image[i] = nanf(""); score[i] = -INFINITY; source[i] = -1;
}

__global__ void feather(float* image, const float* best, const int16_t* source,
    const float* runner_image, const float* runner_score, const int16_t* runner_source,
    int pixels, double width_columns) {
  int i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i >= pixels || runner_source[i] < 0 || abs(source[i]-runner_source[i]) != 1) return;
  double difference = static_cast<double>(best[i])-runner_score[i];
  if (difference >= width_columns) return;
  double weight = .5+difference/(2.*width_columns);
  image[i] = static_cast<float>(static_cast<double>(image[i])*weight+
                               static_cast<double>(runner_image[i])*(1.-weight));
}

__global__ void sample_band(const uint8_t* pixels, const double* axes, const Row* rows,
    const double* xs, const double* native, const double* output, const uint8_t* geometry,
    Flat flat, int width, int nq, int nx, int left, int right, int band,
    float* image, uint16_t* count, int16_t* source, float* best) {
  const int i = blockIdx.x*blockDim.x+threadIdx.x;
  const int span = right-left;
  if (i >= nq*span) return;
  const int q = i/span, column = left+i%span;
  const Row row = rows[q];
  if (!row.supported) return;
  const Sample a = along(row.lower, xs[column], pixels, axes, native, output, geometry, flat, width);
  const Sample b = along(row.upper, xs[column], pixels, axes, native, output, geometry, flat, width);
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

// Pose corrections are evaluated once per recorded encoder row on the CPU.
// These vectors represent the same Ry(pitch) Rx(roll) cylinder rays as D3;
// neither actual vehicle poses nor renderer geometry are accepted by this ABI.
struct GlobalRay {
  double axis, phase, ox, oy, oz, tx, tz, rx, ry, rz;
  int64_t lattice;
};
static_assert(sizeof(GlobalRay) == 88);

__device__ Row global_rows(const GlobalRay* rays, int n, double phase, double footprint) {
  int a = 0, b = n;
  while (a < b) {
    int mid = (a+b)/2;
    if (rays[mid].phase < phase) a = mid+1; else b = mid;
  }
  int left = max(0, a-1), right = min(a, n-1);
  double gap = rays[right].phase-rays[left].phase;
  bool interpolate = rays[right].lattice-rays[left].lattice == 1 &&
      gap <= footprint*(1.+1e-8) && gap > 0. &&
      phase >= rays[left].phase && phase <= rays[right].phase;
  double dl = fabs(phase-rays[left].phase), dr = fabs(phase-rays[right].phase);
  int nearest = dl <= dr ? left : right;
  // Same machine-roundoff centre canonicalization as BandSampler.row_sources.
  if (fmin(dl, dr) <= 8.*2.220446049250313e-16*fmax(1., fabs(phase)))
    interpolate = false;
  return {interpolate ? left : nearest, interpolate ? right : nearest,
          interpolate ? fmin(1., fmax(0., (phase-rays[left].phase)/gap)) : 0.,
          fmin(dl, dr) <= footprint/2.+1e-12, 0};
}

__device__ double2 global_hit(const GlobalRay& r, double x, double radius,
                              double angular_offset = 0., double depth = 0.) {
  double tangent = (x-r.axis)/radius;
  // Rotate inside the recorded row's finite angular footprint, around the
  // fitted scan axis (r.tx, 0, r.tz). This changes neither pose nor source row.
  double rx = r.rx, ry = r.ry, rz = r.rz;
  if (angular_offset != 0.) {
    double cs = cos(angular_offset), sn = sin(angular_offset);
    rx = cs*r.rx+sn*r.tz*r.ry;
    ry = cs*r.ry+sn*(r.tx*r.rz-r.tz*r.rx);
    rz = cs*r.rz-sn*r.tx*r.ry;
  }
  double vx = r.tx*tangent+rx, vy = ry, vz = r.tz*tangent+rz;
  double aa = vy*vy+vz*vz, bb = 2.*(r.oy*vy+r.oz*vz);
  double surface_radius = radius+depth;
  double cc = r.oy*r.oy+r.oz*r.oz-surface_radius*surface_radius;
  double length = (-bb+sqrt(bb*bb-4.*aa*cc))/(2.*aa);
  return make_double2(r.ox+length*vx,
                     radius*atan2(r.oy+length*vy, r.oz+length*vz));
}

__device__ double2 global_forward(const GlobalRay* rays, int n, double x, double q,
                                  double radius, double footprint, double depth, Row* row) {
  *row = global_rows(rays, n, q/radius, footprint);
  double offset = row->lower == row->upper ? q/radius-rays[row->lower].phase : 0.;
  double2 a = global_hit(rays[row->lower], x, radius, offset, depth);
  double2 b = global_hit(rays[row->upper], x, radius, offset, depth);
  return make_double2((1.-row->weight)*a.x+row->weight*b.x,
                      (1.-row->weight)*a.y+row->weight*b.y);
}

__global__ void global_band(const uint8_t* pixels, const double* axes,
    const GlobalRay* rays, int n, const double* qs, const double* xs,
    const double* native, const double* output, const uint8_t* geometry, Flat flat,
    int width, int nq, int nx, int left, int right, int band, double radius,
    double footprint, const double* depths, float* image, uint16_t* count, int16_t* source, float* best,
    float* runner_image, float* runner_score, int16_t* runner_source, double gain) {
  int i = blockIdx.x*blockDim.x+threadIdx.x, span = right-left;
  if (i >= nq*span) return;
  int q_index = i/span, column = left+i%span;
  double target_x = xs[column], target_q = qs[q_index];
  // Shared depth is evaluated at the known OUTPUT point, then held fixed
  // through inversion, exactly as the CPU's inverse_points implementation.
  double depth = depths ? depths[q_index*nx+column] : 0.;
  double x = target_x, q = target_q;
  Row row;
  for (int iteration = 0; iteration < 10; ++iteration) {
    double2 point = global_forward(rays, n, x, q, radius, footprint, depth, &row);
    double ex = point.x-target_x, eq = point.y-target_q;
    x -= ex; q -= eq;
    if (fmax(fabs(ex), fabs(eq)) < 1e-9) break;
  }
  double2 final = global_forward(rays, n, x, q, radius, footprint, depth, &row);
  if (!row.supported || !isfinite(x) || !isfinite(q) || !isfinite(final.x) || !isfinite(final.y) ||
      fmax(fabs(final.x-target_x), fabs(final.y-target_q)) >= 1e-8) return;
  Sample a = along(row.lower, x, pixels, axes, native, output, geometry, flat, width);
  Sample b = along(row.upper, x, pixels, axes, native, output, geometry, flat, width);
  if (!a.valid || !b.valid) return;
  int target = q_index*nx+column;
  ++count[target];
  float score = static_cast<float>(fmin(a.column, width-1.-a.column));
  float value = static_cast<float>(static_cast<double>(a.value)*(1.-row.weight)+
                                   static_cast<double>(b.value)*row.weight);
  if (runner_image) value = static_cast<float>(static_cast<double>(value)*gain);
  if (score > best[target]) {
    if (runner_image) {
      runner_image[target] = image[target]; runner_score[target] = best[target];
      runner_source[target] = source[target];
    }
    image[target] = value;
    best[target] = score;
    source[target] = static_cast<int16_t>(band);
  } else if (runner_image && score > runner_score[target]) {
    runner_image[target] = value; runner_score[target] = score;
    runner_source[target] = static_cast<int16_t>(band);
  }
}
}  // namespace

extern "C" {
int ssb_unroll_abi() { return 2; }
const char* ssb_unroll_error() { return last_error.c_str(); }

void* ssb_unroll_create(int width, const double* native, const double* output,
                        const uint8_t* geometry, const float* flat_offset, const float* flat_gain,
                        const uint8_t* flat_valid) {
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
    for (int i = 0; i < width; ++i)
      if (!std::isfinite(flat_offset[i]) || !std::isfinite(flat_gain[i])) throw std::runtime_error("nonfinite flat calibration");
    context->flat_offset.upload(flat_offset, width*sizeof(float));
    context->flat_gain.upload(flat_gain, width*sizeof(float));
    context->flat_valid.upload(flat_valid, width);
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
    c.use_depth = false; c.fusion = false;
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
int ssb_unroll_band(void* handle, int native_rows, const uint8_t* pixels,
    const double* axes, const void* row_data, int left, int right, int band) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    const auto* rows = static_cast<const Row*>(row_data);
    if (native_rows <= 0 || left < 0 || right > c.columns || right <= left ||
        band < 0 || band >= std::numeric_limits<int16_t>::max())
      throw std::runtime_error("invalid CUDA band dimensions");
    const size_t bytes = static_cast<size_t>(native_rows)*c.width;
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
    const Flat flat{c.flat_offset.as<float>(), c.flat_gain.as<float>(), c.flat_valid.as<uint8_t>()};
    sample_band<<<(n+255)/256,256>>>(c.pixels.as<uint8_t>(), c.axes.as<double>(), c.row_sources.as<Row>(),
        c.xs.as<double>(), c.native_offsets.as<double>(), c.output_offsets.as<double>(),
        c.geometry_valid.as<uint8_t>(), flat, c.width, c.rows, c.columns, left, right, band,
        c.image.as<float>(), c.count.as<uint16_t>(), c.source.as<int16_t>(), c.best.as<float>());
    checked(cudaGetLastError());
  });
}

int ssb_unroll_global_abi() { return 2; }
int ssb_unroll_global_depth(void* handle, const double* depths) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    const size_t n = static_cast<size_t>(c.rows)*c.columns;
    if (!depths || !n) throw std::runtime_error("invalid CUDA global depth tile");
    for (size_t i = 0; i < n; ++i)
      if (!std::isfinite(depths[i]) || depths[i] < 0. || depths[i] > .03+1e-8)
        throw std::runtime_error("invalid CUDA global radial depth");
    size_t total = 0;
    for (auto* b : {&c.native_offsets, &c.output_offsets, &c.geometry_valid, &c.flat_offset,
        &c.flat_gain, &c.flat_valid, &c.pixels, &c.axes, &c.row_sources, &c.xs,
        &c.image, &c.count, &c.source, &c.best, &c.global_rays, &c.global_qs,
        &c.runner_image, &c.runner_score, &c.runner_source}) total += b->capacity;
    if (total+std::max(c.global_depth.capacity, n*sizeof(double)) > kBudget)
      throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
    c.global_depth.upload(depths, n*sizeof(double)); c.account();
    c.use_depth = true;
  });
}
int ssb_unroll_global_band(void* handle, int native_rows, const uint8_t* pixels,
    const double* axes, const void* ray_data, const double* qs, int left, int right,
    int band, double radius, double footprint) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    if (native_rows < 2 || left < 0 || right > c.columns || right <= left ||
        band < 0 || band >= std::numeric_limits<int16_t>::max() ||
        !std::isfinite(radius) || radius <= 0. || !std::isfinite(footprint) || footprint <= 0.)
      throw std::runtime_error("invalid CUDA global band dimensions");
    const auto* rays = static_cast<const GlobalRay*>(ray_data);
    for (int i = 0; i < native_rows; ++i) {
      const double* values = &rays[i].axis;
      for (int j = 0; j < 10; ++j)
        if (!std::isfinite(values[j])) throw std::runtime_error("nonfinite CUDA global ray");
      if (axes[i] != rays[i].axis || (i && (rays[i].phase <= rays[i-1].phase ||
                                          rays[i].lattice <= rays[i-1].lattice)))
        throw std::runtime_error("invalid CUDA global ray ordering");
    }
    for (int i = 0; i < c.rows; ++i)
      if (!std::isfinite(qs[i])) throw std::runtime_error("nonfinite CUDA global grid");
    size_t bytes = static_cast<size_t>(native_rows)*c.width;
    if (bytes > kBudget/2) throw std::runtime_error("CUDA native input exceeds 128 MiB");
    c.reserve_check(bytes, native_rows*sizeof(double), 0);
    // Account for both current retained capacities and the requested new buffers.
    size_t extra = std::max(c.global_rays.capacity, native_rows*sizeof(GlobalRay))-c.global_rays.capacity+
                   std::max(c.global_qs.capacity, c.rows*sizeof(double))-c.global_qs.capacity;
    size_t retained = 0;
    for (auto* b : {&c.native_offsets, &c.output_offsets, &c.geometry_valid, &c.flat_offset,
        &c.flat_gain, &c.flat_valid, &c.xs, &c.image, &c.count, &c.source, &c.best,
        &c.row_sources, &c.global_rays, &c.global_qs, &c.global_depth,
        &c.runner_image, &c.runner_score, &c.runner_source}) retained += b->capacity;
    retained += std::max(c.pixels.capacity, bytes)+std::max(c.axes.capacity, native_rows*sizeof(double));
    if (retained+extra > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
    c.pixels.upload(pixels, bytes); c.axes.upload(axes, native_rows*sizeof(double));
    c.global_rays.upload(rays, native_rows*sizeof(GlobalRay)); c.global_qs.upload(qs, c.rows*sizeof(double));
    c.account();
    Flat flat{c.flat_offset.as<float>(), c.flat_gain.as<float>(), c.flat_valid.as<uint8_t>()};
    int n = c.rows*(right-left);
    global_band<<<(n+255)/256,256>>>(c.pixels.as<uint8_t>(), c.axes.as<double>(),
        c.global_rays.as<GlobalRay>(), native_rows, c.global_qs.as<double>(), c.xs.as<double>(),
        c.native_offsets.as<double>(), c.output_offsets.as<double>(), c.geometry_valid.as<uint8_t>(),
        flat, c.width, c.rows, c.columns, left, right, band, radius, footprint,
        c.use_depth ? c.global_depth.as<double>() : nullptr,
        c.image.as<float>(), c.count.as<uint16_t>(), c.source.as<int16_t>(), c.best.as<float>(),
        c.fusion ? c.runner_image.as<float>() : nullptr,
        c.fusion ? c.runner_score.as<float>() : nullptr,
        c.fusion ? c.runner_source.as<int16_t>() : nullptr, c.fusion_gain);
    checked(cudaGetLastError());
  });
}
int ssb_unroll_fusion_abi() { return 1; }
int ssb_unroll_fusion_begin(void* handle, double width_columns) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    const size_t n = static_cast<size_t>(c.rows)*c.columns;
    if (!n || !std::isfinite(width_columns) || width_columns <= 0. || width_columns > c.width/4.)
      throw std::runtime_error("invalid CUDA feather width");
    size_t extra = std::max(c.runner_image.capacity, n*sizeof(float))-c.runner_image.capacity+
                   std::max(c.runner_score.capacity, n*sizeof(float))-c.runner_score.capacity+
                   std::max(c.runner_source.capacity, n*sizeof(int16_t))-c.runner_source.capacity;
    c.account();
    // account() records the high-water mark; it is also a conservative preallocation bound.
    if (c.peak+extra > kBudget) throw std::runtime_error("CUDA unroll allocation exceeds 256 MiB");
    c.runner_image.grow(n*sizeof(float)); c.runner_score.grow(n*sizeof(float));
    c.runner_source.grow(n*sizeof(int16_t)); c.account();
    c.fusion = true; c.feather_columns = width_columns; c.fusion_gain = 1.;
    initialize_runner<<<(n+255)/256,256>>>(c.runner_image.as<float>(), c.runner_score.as<float>(),
                                       c.runner_source.as<int16_t>(), n);
    checked(cudaGetLastError());
  });
}
int ssb_unroll_fusion_gain(void* handle, double gain) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    if (!c.fusion || !std::isfinite(gain) || gain < 1./1.08-1e-12 || gain > 1.08+1e-12)
      throw std::runtime_error("invalid CUDA bounded radiometric gain");
    c.fusion_gain = gain;
  });
}
int ssb_unroll_finish(void* handle, float* image, uint16_t* count, int16_t* source) {
  return guarded([&] {
    auto& c = *static_cast<Context*>(handle);
    const size_t n = static_cast<size_t>(c.rows)*c.columns;
    if (c.fusion) {
      feather<<<(n+255)/256,256>>>(c.image.as<float>(), c.best.as<float>(), c.source.as<int16_t>(),
          c.runner_image.as<float>(), c.runner_score.as<float>(), c.runner_source.as<int16_t>(),
          n, c.feather_columns);
      checked(cudaGetLastError());
      c.fusion = false;  // finish is repeatable: never feather an already feathered tile.
    }
    checked(cudaDeviceSynchronize());
    checked(cudaMemcpy(image, c.image.pointer, n*sizeof(float), cudaMemcpyDeviceToHost));
    checked(cudaMemcpy(count, c.count.pointer, n*sizeof(uint16_t), cudaMemcpyDeviceToHost));
    checked(cudaMemcpy(source, c.source.pointer, n*sizeof(int16_t), cudaMemcpyDeviceToHost));
  });
}
}  // extern "C"
