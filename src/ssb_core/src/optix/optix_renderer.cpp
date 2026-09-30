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
#include <set>
#include <limits>
#include <chrono>
#include "ssb_core/stage_b_assets.hpp"
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
  std::unique_ptr<StageBAssets> assets;
  std::unique_ptr<CudaSurfaceRecipe> recipe;
  std::vector<const SurfaceTexel*> tile_table;
  struct Slot { SurfaceTexel* data=nullptr; int tile=-1; uint64_t used=0; };
  std::vector<Slot> tile_slots;
  uint64_t cache_clock=0, cache_loads=0, cache_hits=0;
  size_t texture_peak_bytes=0, active_peak_tiles=0;
  double tile_load_seconds=0, footprint_seconds=0, launch_seconds=0;
  struct CpuSlot { std::vector<SurfaceTexel> data; uint64_t used=0; };
  std::map<unsigned,CpuSlot> cpu_tiles;
  uint64_t cpu_clock=0,cpu_cache_hits=0;
  size_t cpu_texture_peak_bytes=0;

  const std::vector<SurfaceTexel>& CpuTile(unsigned tile) {
    auto found=cpu_tiles.find(tile);
    if(found!=cpu_tiles.end()) {found->second.used=++cpu_clock;++cpu_cache_hits;return found->second.data;}
    const size_t slots=assets->cpu_budget/assets->tile_bytes;
    if(cpu_tiles.size()>=slots) {
      auto oldest=std::min_element(cpu_tiles.begin(),cpu_tiles.end(),[](const auto& a,const auto& b){return a.second.used<b.second.used;});
      cpu_tiles.erase(oldest);
    }
    auto inserted=cpu_tiles.emplace(tile,CpuSlot{assets->ReadTile(tile),++cpu_clock}).first;
    cpu_texture_peak_bytes=std::max(cpu_texture_peak_bytes,cpu_tiles.size()*assets->tile_bytes);
    return inserted->second.data;
  }

  void LoadTiles(const std::set<unsigned>& required) {
    if(!assets) return;
    if(required.size()>tile_slots.size()) throw std::runtime_error("Stage B: one row exceeds GPU texture budget");
    active_peak_tiles=std::max(active_peak_tiles,required.size());
    for(auto tile:required) {
      auto found=std::find_if(tile_slots.begin(),tile_slots.end(),[&](const Slot& slot){return slot.tile==int(tile);});
      if(found!=tile_slots.end()) { found->used=++cache_clock; ++cache_hits; continue; }
      Slot* target=nullptr;
      for(auto& slot:tile_slots) if(slot.tile<0 || !required.count(unsigned(slot.tile)))
        if(!target || slot.used<target->used) target=&slot;
      if(!target) throw std::runtime_error("Stage B texture cache cannot evict a required tile");
      if(target->tile>=0) tile_table[target->tile]=nullptr;
      if(!target->data) target->data=Alloc<SurfaceTexel>(assets->tile_bytes/sizeof(SurfaceTexel));
      auto start=std::chrono::steady_clock::now();
      if(recipe)recipe->Generate(tile,target->data,stream);
      else {
        const auto& data=CpuTile(tile);
        SSB_CUDA(cudaMemcpy(target->data,data.data(),assets->tile_bytes,cudaMemcpyHostToDevice));
      }
      tile_load_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      target->tile=tile;target->used=++cache_clock;tile_table[tile]=target->data;++cache_loads;
    }
    size_t allocated=std::count_if(tile_slots.begin(),tile_slots.end(),[](const Slot& slot){return slot.data!=nullptr;});
    texture_peak_bytes=std::max(texture_peak_bytes,allocated*assets->tile_bytes);
    SSB_CUDA(cudaMemcpy(const_cast<SurfaceTexel**>(params.tiles),tile_table.data(),tile_table.size()*sizeof(void*),cudaMemcpyHostToDevice));
  }
  void* d_params = nullptr;
  unsigned char* host_pixels = nullptr;
  unsigned* host_invalid = nullptr;
  std::vector<unsigned> invalid_flags, invalid_columns;
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
  if(!config.optical_scene.empty()) s.assets=std::make_unique<StageBAssets>(config);
  s.params.stage_b=s.assets?1:0;
  s.params.row_stride=s.assets?s.assets->time_samples+1:1;
  s.params.integrated_cracks=s.assets && s.assets->integrated_cracks;
  if(s.assets) {
    const auto& a=*s.assets;
    s.params.area_samples=a.area_samples;s.params.time_samples=a.time_samples;s.params.light_samples=a.light_samples;
    s.params.light_enabled=a.light_enabled;s.params.shadows=a.shadows;
    s.params.adaptive_area=a.adaptive_area;
    s.params.convex_panel_visibility=a.convex_panel_visibility;
    s.params.texture_footprint_samples=a.texture_footprint_samples;s.params.texture_prefilter=a.texture_prefilter;
  }

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
  if(s.assets && s.assets->recipe)s.recipe=std::make_unique<CudaSurfaceRecipe>(*s.assets->recipe);

  const double R = config.tunnel_radius_m, zc = config.tunnel_axis_z_m;
  const unsigned flags = OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;
  OptixBuildInput input = {};
  CUdeviceptr d_geometry = 0;
  if(s.assets) {
    const auto& a=*s.assets;
    static_assert(sizeof(OpticalVertex)==sizeof(float3));
    static_assert(sizeof(OpticalTriangle)==sizeof(uint3));
    s.params.vertices=s.Alloc<float3>(a.vertices.size(),a.vertices.data());
    s.params.triangles=s.Alloc<uint3>(a.triangles.size(),a.triangles.data());
    s.params.face_material=s.Alloc<unsigned>(a.face_material.size(),a.face_material.data());
    d_geometry=reinterpret_cast<CUdeviceptr>(s.params.vertices);
    input.type=OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    input.triangleArray.vertexBuffers=&d_geometry;
    input.triangleArray.numVertices=a.vertices.size();
    input.triangleArray.vertexFormat=OPTIX_VERTEX_FORMAT_FLOAT3;
    input.triangleArray.vertexStrideInBytes=sizeof(float3);
    input.triangleArray.indexBuffer=reinterpret_cast<CUdeviceptr>(s.params.triangles);
    input.triangleArray.numIndexTriplets=a.triangles.size();
    input.triangleArray.indexFormat=OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    input.triangleArray.indexStrideInBytes=sizeof(uint3);
    input.triangleArray.flags=&flags;input.triangleArray.numSbtRecords=1;
  } else {
    // The analytic Stage A cylinder retains its translated row frame.
    OptixAabb box = {-1e4f, float(-R - 1), float(zc - R - 1), 1e4f, float(R + 1), float(zc + R + 1)};
    d_geometry=reinterpret_cast<CUdeviceptr>(s.Alloc<OptixAabb>(1,&box));
    input.type=OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
    input.customPrimitiveArray.aabbBuffers=&d_geometry;input.customPrimitiveArray.numPrimitives=1;
    input.customPrimitiveArray.flags=&flags;input.customPrimitiveArray.numSbtRecords=1;
  }
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
  pc.numPayloadValues = 4;
  pc.numAttributeValues = 2;
  pc.pipelineLaunchParamsVariableName = "params";
  pc.usesPrimitiveTypeFlags = s.assets ? OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE : OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;
  OptixModuleCompileOptions mc = {};
  // These scene/quality settings never change during a renderer's lifetime.
  // Specialising them removes inactive scene paths and dynamic sample loops from
  // the shader; per-row poses, cache addresses and row counts remain unbound.
  std::vector<OptixModuleCompileBoundValueEntry> bound;
#define SSB_BOUND(field) bound.push_back({offsetof(LaunchParams,field),sizeof(s.params.field),&s.params.field,#field})
  SSB_BOUND(stage_b);SSB_BOUND(row_stride);SSB_BOUND(area_samples);SSB_BOUND(time_samples);SSB_BOUND(light_samples);
  SSB_BOUND(light_enabled);SSB_BOUND(shadows);SSB_BOUND(adaptive_area);SSB_BOUND(convex_panel_visibility);SSB_BOUND(integrated_cracks);
  SSB_BOUND(texture_footprint_samples);SSB_BOUND(texture_prefilter);
#undef SSB_BOUND
  mc.boundValues=bound.data();mc.numBoundValues=bound.size();
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
  desc[2].hitgroup.entryFunctionNameCH = s.assets ? "__closesthit__wall" : "__closesthit__tunnel";
  if(!s.assets) {
    desc[2].hitgroup.moduleIS = s.module;
    desc[2].hitgroup.entryFunctionNameIS = "__intersection__tunnel";
  }
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
  s.params.rows = s.Alloc<DeviceRow>(capacity*s.params.row_stride);
  s.params.pixels = s.Alloc<unsigned char>(capacity * config.width);
  s.params.invalid = s.Alloc<unsigned>(capacity);
  s.params.invalid_flags = s.Alloc<unsigned>(capacity);
  s.params.invalid_column = s.Alloc<unsigned>(capacity);
  s.invalid_flags.resize(capacity);s.invalid_columns.resize(capacity);
  s.params.debug_hits = s.Alloc<double>(std::max<size_t>(1, capacity * s.debug_columns.size() * 2));
  if(s.assets) {
    const auto& a=*s.assets;
    s.tile_table.resize(a.nx*a.nq,nullptr);s.tile_slots.resize(a.cache_slots);
    s.params.tiles=s.Alloc<const SurfaceTexel*>(s.tile_table.size(),s.tile_table.data());
    s.params.tile_core=a.core;s.params.tile_gutter=a.gutter;s.params.tile_side=a.side;
    s.params.tiles_x=a.nx;s.params.tiles_q=a.nq;s.params.pixels_x=a.pixel_x;s.params.pixels_q=a.pixel_q;
    s.params.tex_x0=a.x0;s.params.tex_q0=a.q0;s.params.tex_dx=a.dx;s.params.tex_dq=a.dq;s.params.tex_period=a.period;
    s.params.cracks=s.Alloc<CrackSegment>(std::max<size_t>(1,a.segments.size()),a.segments.empty()?nullptr:a.segments.data());
    s.params.crack_offsets=s.Alloc<unsigned>(a.offsets.size(),a.offsets.data());
    s.params.crack_indices=s.Alloc<unsigned>(std::max<size_t>(1,a.indices.size()),a.indices.empty()?nullptr:a.indices.data());
    s.params.crack_x0=a.crack_x0;s.params.crack_q0=a.crack_q0;s.params.crack_cell=a.crack_cell;
    s.params.crack_nx=a.crack_nx;s.params.crack_nq=a.crack_nq;
    s.params.crack_interior=a.crack_interior;s.params.crack_interior_variation=a.crack_interior_variation;
    s.params.crack_edge_band=a.crack_edge_band;s.params.crack_edge_darkening=a.crack_edge_darkening;
    s.params.groove_albedo=a.groove_albedo;s.params.groove_detail_contrast=a.groove_detail_contrast;s.params.gap_albedo=a.gap_albedo;
    if(!a.filler.empty()) {
      s.params.filler=s.Alloc<unsigned short>(a.filler.size(),a.filler.data());
      s.params.filler_width=a.filler_width;s.params.filler_height=a.filler_height;s.params.filler_pitch=a.filler_pitch;
      s.params.filler_scale=a.filler_scale;s.params.filler_mean=a.filler_mean;s.params.filler_roughness=a.filler_roughness;
    }
    s.params.area_samples=a.area_samples;s.params.time_samples=a.time_samples;s.params.light_samples=a.light_samples;
    s.params.pixel_step=config.pixel_pitch_m/config.FocalLength();s.params.response_gain=a.response_gain;
    s.params.light_enabled=a.light_enabled;s.params.shadows=a.shadows;s.params.lamp_length=a.lamp_length;
    s.params.adaptive_area=a.adaptive_area;
    s.params.integrated_cracks=a.integrated_cracks;
    if(a.integrated_cracks)s.params.critical_edges=s.Alloc<unsigned>(a.critical_edges.size(),a.critical_edges.data());
    s.params.convex_panel_visibility=a.convex_panel_visibility;
    s.params.footprint_x=a.footprint_x;s.params.footprint_q=a.footprint_q;
    s.params.lamp_tangential=a.lamp_tangential;s.params.lamp_radial=a.lamp_radial;
    s.params.lamp_axial=a.lamp_axial;s.params.lamp_width=a.lamp_width;
  }
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
  std::vector<DeviceRow> rows(n*s.params.row_stride);
  std::vector<std::set<unsigned>> footprints(s.assets?n:0);
  auto prep_start=std::chrono::steady_clock::now();
  for (size_t i = 0; i < n; ++i) for(unsigned sample=0;sample<s.params.row_stride;++sample) {
    PoseSample pose=jobs[i].pose;
    if(sample) {
      // Linear local motion over an 8 us exposure, using the actual interpolated
      // velocity and scan speed. Acceleration terms are not inferred from commands.
      double dt=((sample-.5)/s.assets->time_samples-.5)*s.config.exposure_s;
      pose.x+=pose.v*dt;pose.theta+=pose.omega*dt;
    }
    const HeadPose h=TrueHeadPose(s.config,pose);
    DeviceRow& r=rows[i*s.params.row_stride+sample];
    r.origin_x=h.origin[0];r.origin_y=h.origin[1];r.origin_z=h.origin[2];
    for(int k=0;k<3;++k) {r.optical[k]=h.optical[k];r.line[k]=h.line[k];r.scan[k]=h.scan[k];}
    r.optical_q=s.config.tunnel_radius_m*std::atan2(double(r.optical[1]),double(r.optical[2]));
    if(s.assets) {
      const auto required=s.assets->Footprint(h,s.params.pixel_step,std::abs(s.config.PixelTangent(0)));
      footprints[i].insert(required.begin(),required.end());
    }
  }
  s.footprint_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-prep_start).count();
  pixels.resize(n*s.params.width);hits.resize(n*s.debug_columns.size()*2);
  size_t first=0;
  while(first<n) {
    size_t end=first;std::set<unsigned> required;
    while(end<n) {
      if(s.assets) {
        auto combined=required;combined.insert(footprints[end].begin(),footprints[end].end());
        if(combined.size()>s.assets->cache_slots) break;
        required=std::move(combined);
      }
      ++end;
    }
    if(end==first) throw std::runtime_error("Stage B row footprint exceeds texture budget");
    s.LoadTiles(required);
    const size_t count=end-first, hit_values=count*s.debug_columns.size()*2;
    s.params.row_count=count;
    auto launch_start=std::chrono::steady_clock::now();
    SSB_CUDA(cudaMemcpyAsync(const_cast<DeviceRow*>(s.params.rows),rows.data()+first*s.params.row_stride,
                              count*s.params.row_stride*sizeof(DeviceRow),cudaMemcpyHostToDevice,s.stream));
    SSB_CUDA(cudaMemsetAsync(s.params.invalid,0,count*sizeof(unsigned),s.stream));
    SSB_CUDA(cudaMemsetAsync(s.params.invalid_flags,0,count*sizeof(unsigned),s.stream));
    SSB_CUDA(cudaMemsetAsync(s.params.invalid_column,255,count*sizeof(unsigned),s.stream));
    SSB_CUDA(cudaMemcpyAsync(s.d_params,&s.params,sizeof(LaunchParams),cudaMemcpyHostToDevice,s.stream));
    SSB_OPTIX(optixLaunch(s.pipeline,s.stream,reinterpret_cast<CUdeviceptr>(s.d_params),sizeof(LaunchParams),
                          &s.sbt,s.params.width,count,1));
    SSB_CUDA(cudaMemcpyAsync(s.host_pixels,s.params.pixels,count*s.params.width,cudaMemcpyDeviceToHost,s.stream));
    SSB_CUDA(cudaMemcpyAsync(s.host_invalid,s.params.invalid,count*sizeof(unsigned),cudaMemcpyDeviceToHost,s.stream));
    SSB_CUDA(cudaMemcpyAsync(s.invalid_flags.data(),s.params.invalid_flags,count*sizeof(unsigned),cudaMemcpyDeviceToHost,s.stream));
    SSB_CUDA(cudaMemcpyAsync(s.invalid_columns.data(),s.params.invalid_column,count*sizeof(unsigned),cudaMemcpyDeviceToHost,s.stream));
    if(hit_values) SSB_CUDA(cudaMemcpyAsync(s.host_hits,s.params.debug_hits,hit_values*sizeof(double),cudaMemcpyDeviceToHost,s.stream));
    SSB_CUDA(cudaStreamSynchronize(s.stream));
    s.launch_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-launch_start).count();
    for(size_t i=0;i<count;++i) if(s.host_invalid[i])
      throw std::runtime_error("OptiX batch rejected: row sequence "+std::to_string(jobs[first+i].record.sequence)+
                               " has "+std::to_string(s.host_invalid[i])+" invalid wall/cache samples, flags="+
                               std::to_string(s.invalid_flags[i])+", column="+std::to_string(s.invalid_columns[i])+
                               ", x="+std::to_string(jobs[first+i].pose.x)+", theta="+std::to_string(jobs[first+i].pose.theta));
    std::copy_n(s.host_pixels,count*s.params.width,pixels.data()+first*s.params.width);
    if(hit_values) std::copy_n(s.host_hits,hit_values,hits.data()+first*s.debug_columns.size()*2);
    first=end;
  }
}

nlohmann::json OptixRenderer::Describe() const {
  const auto& s = *impl_;
  const char* runtime = std::getenv("SSB_OPTIX_RUNTIME");
  return {{"backend", "optix"},
          {"scene", s.assets ? "textured_triangle_tunnel.stage_b" : "analytic_cylinder.stage_a"},
          {"optical_scene_sha256", s.assets?s.assets->scene_hash:""},
          {"surface_sha256", s.assets?s.assets->surface_hash:""},
          {"defects_sha256", s.assets?s.assets->defect_hash:""},
          {"gpu_texture_budget_bytes", s.assets?s.assets->gpu_budget:0},
          {"cpu_texture_budget_bytes", s.assets?s.assets->cpu_budget:0},
          {"cpu_texture_allocated_peak_bytes", s.cpu_texture_peak_bytes}, {"cpu_tile_cache_hits", s.cpu_cache_hits},
          {"texture_allocated_peak_bytes", s.texture_peak_bytes},
          {"active_texture_peak_tiles", s.active_peak_tiles},
          {"tile_loads", s.cache_loads}, {"tile_hits", s.cache_hits}, {"tile_load_seconds", s.tile_load_seconds},
          {"footprint_seconds", s.footprint_seconds}, {"launch_seconds", s.launch_seconds},
          {"runtime_surface_recipe",bool(s.recipe)},
          {"recipe_source_device_bytes",s.recipe?s.recipe->Bytes():0},
          {"area_axis_samples", s.params.area_samples}, {"exposure_time_samples", s.params.time_samples},
          {"convex_panel_visibility",bool(s.params.convex_panel_visibility)},
          {"adaptive_area",bool(s.params.adaptive_area)},
          {"integrated_cracks",bool(s.params.integrated_cracks)},
          {"texture_footprint_samples",s.params.texture_footprint_samples},
          {"joint_filler_texture",bool(s.params.filler)},{"texture_prefilter",bool(s.params.texture_prefilter)},
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

nlohmann::json OptixRenderer::EvaluationAssets() const {
  const auto& s=*impl_;
  if(!s.assets) return nullptr;
  const auto& a=*s.assets;
  return {{"schema","ssb.optical_archive.v1"},{"scene",a.scene},{"surface",a.surface},{"defects",a.defects},
          {"scene_sha256",a.scene_hash},{"surface_sha256",a.surface_hash},{"defects_sha256",a.defect_hash}};
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
  if(s.assets) {
    double worst=0;
    for(size_t i=0;i<s.debug_columns.size();++i) {
      double x,q;
      if(!s.assets->CpuHit(h,c.PixelTangent(s.debug_columns[i]),0,&x,&q))
        throw std::runtime_error("Stage B self-check: CPU triangle miss");
      worst=std::max({worst,std::abs(hits[2*i]-x),std::abs(hits[2*i+1]-q)});
    }
    auto range=std::minmax_element(pixels.begin(),pixels.end());
    bool ok=worst<5e-6 && *range.second>0;
    nlohmann::json result={{"passed",ok},{"max_debug_hit_error_m",worst},{"min_code",*range.first},
                           {"max_code",*range.second},{"geometry_reference","independent double triangle intersections"},
                           {"radiometry_reference","not covered by this self-check; separate optical tests required"}};
    if(!ok) throw std::runtime_error("Stage B self-check failed: "+result.dump());
    return result;
  }
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
