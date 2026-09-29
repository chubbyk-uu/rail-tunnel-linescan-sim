#include "ssb_core/optix_renderer.hpp"

#include <cuda_runtime.h>
#include <dlfcn.h>
#include <link.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include "launch_params.h"
#include "ssb_core/camera_model.hpp"
#include "ssb_core/sha256.hpp"
#include "ssb_core/texture.hpp"

#define SSB_CUDA(x)                                                                                  \
  do {                                                                                               \
    const cudaError_t e = (x);                                                                       \
    if (e != cudaSuccess) throw std::runtime_error(std::string(#x) + ": " + cudaGetErrorString(e));   \
  } while (0)
#define SSB_OPTIX(x)                                                                                 \
  do {                                                                                               \
    const OptixResult e = (x);                                                                       \
    if (e != OPTIX_SUCCESS)                                                                          \
      throw std::runtime_error(std::string(#x) + ": " + optixGetErrorName(e));                       \
  } while (0)

namespace ssb {
namespace {

struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) Record {
  char header[OPTIX_SBT_RECORD_HEADER_SIZE];
};

std::string LoadedLibraryPath(const char* soname) {
  void* handle = dlopen(soname, RTLD_NOW | RTLD_NOLOAD);
  if (!handle) return "";
  link_map* map = nullptr;
  std::string path;
  if (dlinfo(handle, RTLD_DI_LINKMAP, &map) == 0 && map && map->l_name) path = map->l_name;
  dlclose(handle);
  return path;
}

// Host double-precision analytic hit, for the functional self-check only.
bool AnalyticHit(const Config& c, const HeadPose& h, double tan_u, double* x, double* q) {
  Vec3 d;
  for (int i = 0; i < 3; ++i) d[i] = h.optical[i] + tan_u * h.line[i];
  const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  for (auto& v : d) v /= n;
  const double oy = h.origin[1], oz = h.origin[2] - c.tunnel_axis_z_m;
  const double a = d[1] * d[1] + d[2] * d[2], b = 2 * (oy * d[1] + oz * d[2]);
  const double cc = oy * oy + oz * oz - c.tunnel_radius_m * c.tunnel_radius_m;
  const double disc = b * b - 4 * a * cc;
  if (a <= 0 || disc < 0) return false;
  const double t = (-b + std::sqrt(disc)) / (2 * a);
  *x = h.origin[0] + t * d[0];
  *q = c.tunnel_radius_m * std::atan2(oy + t * d[1], oz + t * d[2]);
  return true;
}

}  // namespace

std::filesystem::path DefaultPtxPath() {
  if (const char* env = std::getenv("SSB_PTX")) return env;
  return SSB_PTX_INSTALL_PATH;
}

struct OptixRenderer::Impl {
  Config config;
  size_t capacity = 0;
  std::filesystem::path ptx_path;
  std::string ptx_sha256;
  std::vector<int> debug_columns;
  OptixDeviceContext context = nullptr;
  OptixModule module = nullptr;
  OptixProgramGroup groups[3] = {};
  OptixPipeline pipeline = nullptr;
  OptixShaderBindingTable sbt = {};
  cudaStream_t stream = nullptr;
  std::vector<void*> allocations;
  LaunchParams params = {};
  void* d_params = nullptr;
  unsigned char* host_pixels = nullptr;
  unsigned* host_invalid = nullptr;
  double* host_hits = nullptr;
  cudaDeviceProp device = {};
  int driver_version = 0, runtime_version = 0;

  template <class T>
  T* Alloc(size_t count, const void* init = nullptr) {
    void* p = nullptr;
    SSB_CUDA(cudaMalloc(&p, count * sizeof(T)));
    allocations.push_back(p);
    if (init) SSB_CUDA(cudaMemcpy(p, init, count * sizeof(T), cudaMemcpyHostToDevice));
    return static_cast<T*>(p);
  }

  ~Impl() {
    if (stream) cudaStreamSynchronize(stream);
    if (pipeline) optixPipelineDestroy(pipeline);
    for (auto g : groups)
      if (g) optixProgramGroupDestroy(g);
    if (module) optixModuleDestroy(module);
    for (void* p : allocations) cudaFree(p);
    if (host_pixels) cudaFreeHost(host_pixels);
    if (host_invalid) cudaFreeHost(host_invalid);
    if (host_hits) cudaFreeHost(host_hits);
    if (stream) cudaStreamDestroy(stream);
    if (context) optixDeviceContextDestroy(context);
  }
};

OptixRenderer::OptixRenderer(const Config& config, const std::filesystem::path& ptx, size_t capacity)
    : impl_(std::make_unique<Impl>()) {
  auto& s = *impl_;
  config.Validate();
  if (capacity == 0 || capacity > 16384) throw std::invalid_argument("renderer capacity must be in [1, 16384]");
  s.config = config;
  s.capacity = capacity;
  s.ptx_path = ptx;
  s.debug_columns = DebugColumns(config);

  std::ifstream in(ptx, std::ios::binary);
  const std::string code((std::istreambuf_iterator<char>(in)), {});
  if (code.empty()) throw std::runtime_error("cannot read OptiX PTX: " + ptx.string());
  s.ptx_sha256 = Sha256Hex(code.data(), code.size());

  SSB_CUDA(cudaFree(nullptr));
  int dev = 0;
  SSB_CUDA(cudaGetDevice(&dev));
  SSB_CUDA(cudaGetDeviceProperties(&s.device, dev));
  SSB_CUDA(cudaDriverGetVersion(&s.driver_version));
  SSB_CUDA(cudaRuntimeGetVersion(&s.runtime_version));
  SSB_OPTIX(optixInit());
  OptixDeviceContextOptions options = {};
  SSB_OPTIX(optixDeviceContextCreate(nullptr, &options, &s.context));
  SSB_CUDA(cudaStreamCreate(&s.stream));

  // One analytic tunnel primitive. Rays are traced in the row frame (x shifted), so
  // its box spans x widely; the true x range is enforced after the hit, in double.
  const double R = config.tunnel_radius_m, zc = config.tunnel_axis_z_m;
  OptixAabb box = {-1e4f, float(-R - 1), float(zc - R - 1), 1e4f, float(R + 1), float(zc + R + 1)};
  CUdeviceptr d_box = reinterpret_cast<CUdeviceptr>(s.Alloc<OptixAabb>(1, &box));
  const unsigned flags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
  OptixBuildInput input = {};
  input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
  input.customPrimitiveArray.aabbBuffers = &d_box;
  input.customPrimitiveArray.numPrimitives = 1;
  input.customPrimitiveArray.flags = &flags;
  input.customPrimitiveArray.numSbtRecords = 1;
  OptixAccelBuildOptions accel = {};
  accel.buildFlags = OPTIX_BUILD_FLAG_NONE;
  accel.operation = OPTIX_BUILD_OPERATION_BUILD;
  OptixAccelBufferSizes sizes;
  SSB_OPTIX(optixAccelComputeMemoryUsage(s.context, &accel, &input, 1, &sizes));
  auto* temp = s.Alloc<unsigned char>(sizes.tempSizeInBytes);
  auto* gas = s.Alloc<unsigned char>(sizes.outputSizeInBytes);
  SSB_OPTIX(optixAccelBuild(s.context, s.stream, &accel, &input, 1, reinterpret_cast<CUdeviceptr>(temp),
                            sizes.tempSizeInBytes, reinterpret_cast<CUdeviceptr>(gas), sizes.outputSizeInBytes,
                            &s.params.handle, nullptr, 0));
  SSB_CUDA(cudaStreamSynchronize(s.stream));

  OptixPipelineCompileOptions pc = {};
  pc.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
  pc.numPayloadValues = 3;
  pc.numAttributeValues = 2;
  pc.pipelineLaunchParamsVariableName = "params";
  pc.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;
  OptixModuleCompileOptions mc = {};
  char log[8192];
  size_t log_size = sizeof(log);
  SSB_OPTIX(optixModuleCreate(s.context, &mc, &pc, code.data(), code.size(), log, &log_size, &s.module));
  OptixProgramGroupDesc desc[3] = {};
  desc[0].kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
  desc[0].raygen.module = s.module;
  desc[0].raygen.entryFunctionName = "__raygen__scan";
  desc[1].kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
  desc[1].miss.module = s.module;
  desc[1].miss.entryFunctionName = "__miss__primary";
  desc[2].kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
  desc[2].hitgroup.moduleCH = s.module;
  desc[2].hitgroup.entryFunctionNameCH = "__closesthit__tunnel";
  desc[2].hitgroup.moduleIS = s.module;
  desc[2].hitgroup.entryFunctionNameIS = "__intersection__tunnel";
  OptixProgramGroupOptions go = {};
  log_size = sizeof(log);
  SSB_OPTIX(optixProgramGroupCreate(s.context, desc, 3, &go, log, &log_size, s.groups));
  OptixPipelineLinkOptions pl = {};
  pl.maxTraceDepth = 1;
  log_size = sizeof(log);
  SSB_OPTIX(optixPipelineCreate(s.context, &pc, &pl, s.groups, 3, log, &log_size, &s.pipeline));
  OptixStackSizes stack = {};
  for (auto g : s.groups) SSB_OPTIX(optixUtilAccumulateStackSizes(g, &stack, s.pipeline));
  unsigned a, b, c;
  SSB_OPTIX(optixUtilComputeStackSizes(&stack, 1, 0, 0, &a, &b, &c));
  SSB_OPTIX(optixPipelineSetStackSize(s.pipeline, a, b, c, 1));
  Record records[3] = {};
  for (int i = 0; i < 3; ++i) SSB_OPTIX(optixSbtRecordPackHeader(s.groups[i], records + i));
  auto* d_records = s.Alloc<Record>(3, records);
  const auto base = reinterpret_cast<CUdeviceptr>(d_records);
  s.sbt.raygenRecord = base;
  s.sbt.missRecordBase = base + sizeof(Record);
  s.sbt.missRecordCount = 1;
  s.sbt.missRecordStrideInBytes = sizeof(Record);
  s.sbt.hitgroupRecordBase = base + 2 * sizeof(Record);
  s.sbt.hitgroupRecordCount = 1;
  s.sbt.hitgroupRecordStrideInBytes = sizeof(Record);

  std::vector<float> tangents(config.width);
  for (int u = 0; u < config.width; ++u) tangents[u] = static_cast<float>(config.PixelTangent(u));
  std::vector<int> slot(config.width, -1);
  for (size_t i = 0; i < s.debug_columns.size(); ++i) slot[s.debug_columns[i]] = static_cast<int>(i);
  s.params.width = config.width;
  s.params.radius = R;
  s.params.axis_z = zc;
  s.params.x_min = config.tunnel_x_min_m;
  s.params.x_max = config.tunnel_x_max_m;
  s.params.tangents = s.Alloc<float>(tangents.size(), tangents.data());
  s.params.debug_slot = s.Alloc<int>(slot.size(), slot.data());
  s.params.debug_count = static_cast<unsigned>(s.debug_columns.size());
  s.params.rows = s.Alloc<DeviceRow>(capacity);
  s.params.pixels = s.Alloc<unsigned char>(capacity * config.width);
  s.params.invalid = s.Alloc<unsigned>(capacity);
  s.params.debug_hits = s.Alloc<double>(std::max<size_t>(1, capacity * s.debug_columns.size() * 2));
  s.d_params = s.Alloc<LaunchParams>(1);
  SSB_CUDA(cudaMallocHost(reinterpret_cast<void**>(&s.host_pixels), capacity * config.width));
  SSB_CUDA(cudaMallocHost(reinterpret_cast<void**>(&s.host_invalid), capacity * sizeof(unsigned)));
  SSB_CUDA(cudaMallocHost(reinterpret_cast<void**>(&s.host_hits),
                          std::max<size_t>(1, capacity * s.debug_columns.size() * 2) * sizeof(double)));
}

OptixRenderer::~OptixRenderer() = default;

void OptixRenderer::Render(const std::vector<RowJob>& jobs, std::vector<uint8_t>& pixels,
                           std::vector<double>& hits) {
  auto& s = *impl_;
  const size_t n = jobs.size();
  if (n == 0 || n > s.capacity) throw std::invalid_argument("render batch size out of range");
  std::vector<DeviceRow> rows(n);
  for (size_t i = 0; i < n; ++i) {
    const HeadPose h = TrueHeadPose(s.config, jobs[i].pose);
    DeviceRow& r = rows[i];
    r.origin_x = h.origin[0];
    r.origin_y = static_cast<float>(h.origin[1]);
    r.origin_z = static_cast<float>(h.origin[2]);
    for (int k = 0; k < 3; ++k) {
      r.optical[k] = static_cast<float>(h.optical[k]);
      r.line[k] = static_cast<float>(h.line[k]);
    }
  }
  s.params.row_count = static_cast<unsigned>(n);
  const size_t hit_values = n * s.debug_columns.size() * 2;
  SSB_CUDA(cudaMemcpyAsync(const_cast<DeviceRow*>(s.params.rows), rows.data(), n * sizeof(DeviceRow),
                           cudaMemcpyHostToDevice, s.stream));
  SSB_CUDA(cudaMemsetAsync(s.params.invalid, 0, n * sizeof(unsigned), s.stream));
  SSB_CUDA(cudaMemcpyAsync(s.d_params, &s.params, sizeof(LaunchParams), cudaMemcpyHostToDevice, s.stream));
  SSB_OPTIX(optixLaunch(s.pipeline, s.stream, reinterpret_cast<CUdeviceptr>(s.d_params), sizeof(LaunchParams),
                        &s.sbt, s.params.width, static_cast<unsigned>(n), 1));
  SSB_CUDA(cudaMemcpyAsync(s.host_pixels, s.params.pixels, n * s.params.width, cudaMemcpyDeviceToHost, s.stream));
  SSB_CUDA(cudaMemcpyAsync(s.host_invalid, s.params.invalid, n * sizeof(unsigned), cudaMemcpyDeviceToHost, s.stream));
  if (hit_values)
    SSB_CUDA(cudaMemcpyAsync(s.host_hits, s.params.debug_hits, hit_values * sizeof(double), cudaMemcpyDeviceToHost,
                             s.stream));
  SSB_CUDA(cudaStreamSynchronize(s.stream));
  for (size_t i = 0; i < n; ++i)
    if (s.host_invalid[i])
      throw std::runtime_error("OptiX batch rejected: row sequence " + std::to_string(jobs[i].record.sequence) +
                               " has " + std::to_string(s.host_invalid[i]) + " rays off the tunnel wall");
  pixels.assign(s.host_pixels, s.host_pixels + n * s.params.width);
  hits.assign(s.host_hits, s.host_hits + hit_values);
}

nlohmann::json OptixRenderer::Describe() const {
  const auto& s = *impl_;
  const char* runtime = std::getenv("SSB_OPTIX_RUNTIME");
  return {{"backend", "optix"},
          {"scene", "analytic_cylinder.stage_a"},
          {"optix_abi_version", OPTIX_VERSION},
          {"device", s.device.name},
          {"compute_capability", std::to_string(s.device.major) + "." + std::to_string(s.device.minor)},
          {"cuda_driver_version", s.driver_version},
          {"cuda_runtime_version", s.runtime_version},
          {"optix_library", LoadedLibraryPath("libnvoptix.so.1")},
          {"cuda_library", LoadedLibraryPath("libcuda.so.1")},
          {"optix_runtime_env", runtime ? runtime : ""},
          {"ptx_path", s.ptx_path.string()},
          {"ptx_sha256", s.ptx_sha256},
          {"capacity_rows", s.capacity},
          {"debug_columns", s.debug_columns}};
}

nlohmann::json OptixRenderer::SelfCheck() {
  auto& s = *impl_;
  const Config& c = s.config;
  RowJob job{};
  // Generic probe: not on a texture cell edge (theta = 0.3 put the whole line on a
  // 1 mm edge, where 1e-12 m decides the cell).
  job.pose.x = 0.5 * (c.tunnel_x_min_m + c.tunnel_x_max_m) - c.truth.head_mount_x_m + 0.01234;
  job.pose.theta = 0.3123;
  std::vector<uint8_t> pixels;
  std::vector<double> hits;
  Render({job}, pixels, hits);
  const HeadPose h = TrueHeadPose(c, job.pose);
  int mismatched = 0;
  for (int u = 0; u < c.width; ++u) {
    double x, q;
    if (!AnalyticHit(c, h, c.PixelTangent(u), &x, &q)) throw std::runtime_error("self-check: analytic miss");
    mismatched += pixels[u] != AlbedoCode(WallAlbedo(x, q));
  }
  double max_hit_error = 0;
  for (size_t i = 0; i < s.debug_columns.size(); ++i) {
    double x, q;
    AnalyticHit(c, h, c.PixelTangent(s.debug_columns[i]), &x, &q);
    max_hit_error = std::max({max_hit_error, std::abs(hits[2 * i] - x), std::abs(hits[2 * i + 1] - q)});
  }
  // Float ray directions may move a hit across a 1 mm noise cell edge on a few pixels.
  const bool ok = mismatched <= c.width / 100 && max_hit_error < 1e-5;
  nlohmann::json result = {{"probe_theta_rad", job.pose.theta},
                           {"pixels_differing_from_host", mismatched},
                           {"max_debug_hit_error_m", max_hit_error},
                           {"passed", ok}};
  if (!ok) throw std::runtime_error("OptiX self-check failed: " + result.dump());
  return result;
}

}  // namespace ssb
