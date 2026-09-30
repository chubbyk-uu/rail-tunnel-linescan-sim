// Deterministic local quilt replay, following 4WIDS runtime material architecture.
// Preserves this project's mono16, rough8 and signed normal16 channels.
#include "ssb_core/surface_recipe.hpp"
#include <cuda_runtime.h>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace ssb {
namespace {
void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(std::string("CUDA surface recipe: ")+cudaGetErrorString(e));}
__device__ float4 Mix(float4 a,float4 b,float w){return make_float4(a.x+(b.x-a.x)*w,a.y+(b.y-a.y)*w,a.z+(b.z-a.z)*w,a.w+(b.w-a.w)*w);}
// Fixed 1/32 bilinear position (same rounding as Source/Alpha): integer cell and weight.
__device__ int2 Fixed32(double x){int u=int(floor(x*32+.5)),x0=int(floor(double(u)/32));return make_int2(x0,u-x0*32);}
__device__ float4 SourceAt(const RecipeSource& s,int2 u,int2 v){
  int a=0,b=0,c=0,d=0;
  for(int j=0;j<2;++j)for(int i=0;i<2;++i){
    auto t=s.data[size_t(max(0,min(s.height-1,v.x+j)))*s.width+max(0,min(s.width-1,u.x+i))];
    int w=(i?u.y:32-u.y)*(j?v.y:32-v.y);a+=int(t.albedo)*w;b+=int(t.roughness)*w;c+=int(t.nx)*w;d+=int(t.nq)*w;
  }return make_float4(a/(1024.f*65535.f),b/(1024.f*255.f),c/(1024.f*32767.f),d/(1024.f*32767.f));
}
__device__ float AlphaAt(const unsigned char* mask,int side,int2 u,int2 v){
  int sum=0;
  for(int j=0;j<2;++j)for(int i=0;i<2;++i)sum+=mask[max(0,min(side-1,v.x+j))*side+max(0,min(side-1,u.x+i))]*(i?u.y:32-u.y)*(j?v.y:32-v.y);
  return sum/(1024.f*255.f);
}
constexpr int kBlock=16,kLine=256,kMaxJobs=64;
// Orientation matrices are integer (entries 0/±1), so a patch's alpha and source coordinates
// are separable: u,v each depend on the column or the row alone (adding the exact 0 term does
// not change the double result). The double-precision work is therefore done once per tile
// column/row (Coords) and per patch column/row (Axes) instead of per texel; texel values are
// unchanged.
struct Job { unsigned tile; int first, count; SurfaceTexel* out; };
struct Slot { int id, job; };                       // one selected patch of one job
struct Coord { double g, f; int2 m; };              // guide coordinate; macro weight and cells
struct Axis { int inside; int2 alpha, source; };
__global__ void Coords(RecipeGrid g,const Job* jobs,RecipeMacro macro,Coord* coords){
  int i=blockIdx.x*kLine+threadIdx.x,side=g.core+2*g.gutter;bool column=blockIdx.y==0;if(i>=side)return;
  unsigned tile=jobs[blockIdx.z].tile;Coord c{};
  if(column){double xp=fmax(.5,fmin(g.pixel_x-.5,double(tile%g.nx*g.core)+i-g.gutter+.5));c.g=(g.x0+xp*g.dx-g.guide_x0)/g.guide_step;
    if(macro.data){double u=(g.guide_x0+c.g*g.guide_step-macro.x0)/macro.pitch-.5;int x0=int(floor(u));c.f=u-x0;
      c.m=make_int2(max(0,min(macro.width-1,x0)),max(0,min(macro.width-1,x0+1)));}}
  else{double qp=fmod(double(tile/g.nx*g.core)+i-g.gutter+.5,double(g.pixel_q));if(qp<0)qp+=g.pixel_q;c.g=(g.q0+qp*g.dq-g.guide_q0)/g.guide_step;
    if(macro.data){double v=fmod((g.guide_q0+c.g*g.guide_step-macro.q0)/macro.pitch,double(macro.height));if(v<0)v+=macro.height;v-=.5;
      int y0=int(floor(v));c.f=v-y0;c.m=make_int2((y0%macro.height+macro.height)%macro.height,((y0+1)%macro.height+macro.height)%macro.height);}}
  coords[(size_t(blockIdx.z)*2+blockIdx.y)*side+i]=c;
}
__global__ void Axes(RecipeGrid g,const Slot* slots,const RecipeSource* sources,const RecipePatch* patches,const Coord* coords,Axis* axes){
  int i=blockIdx.x*kLine+threadIdx.x,side=g.core+2*g.gutter;bool column=blockIdx.y==0;if(i>=side)return;
  Slot slot=slots[blockIdx.z];auto p=patches[slot.id];const auto& src=sources[p.source];
  double pv=coords[(size_t(slot.job)*2+blockIdx.y)*side+i].g-(column?p.left:p.top);
  Axis a;a.inside=pv>=0&&pv<g.patch_side;a.alpha=Fixed32(pv-.5);
  // Column: u uses m00 (v uses m10); row: u uses m01 (v uses m11). Exactly one is nonzero per axis.
  int mu=column?p.m00:p.m01,mv=column?p.m10:p.m11;
  a.source=mu?Fixed32((mu*pv*g.guide_step+0.+p.ox)/src.native-.5):Fixed32((mv*pv*g.guide_step+0.+p.oq)/src.native-.5);
  axes[(size_t(blockIdx.z)*2+blockIdx.y)*side+i]=a;
}
__global__ void GenerateTile(RecipeGrid g,const Job* jobs,const Slot* slots,const RecipeSource* sources,const RecipePatch* patches,
 const unsigned char* alpha,const Coord* coords,const Axis* axes,RecipeMacro macro,unsigned* invalid){
  int col=blockIdx.x*kBlock+threadIdx.x,row=blockIdx.y*kBlock+threadIdx.y,side=g.core+2*g.gutter;
  if(col>=side||row>=side)return;
  const Job job=jobs[blockIdx.z];
  float4 value=make_float4(0,0,0,0);float filled=0;
  for(int k=job.first;k<job.first+job.count;++k){
    Axis c=axes[size_t(k)*2*side+col],r=axes[(size_t(k)*2+1)*side+row];if(!c.inside||!r.inside)continue;
    int id=slots[k].id;auto p=patches[id];const auto& src=sources[p.source];
    float a=AlphaAt(alpha+size_t(id)*g.patch_side*g.patch_side,g.patch_side,c.alpha,r.alpha);
    // The column axis carries u when m00!=0, otherwise v (and the row axis the other).
    auto t=p.m00?SourceAt(src,c.source,r.source):SourceAt(src,r.source,c.source);
    float z=t.z;t.z=z*p.m00+t.w*p.m10;t.w=z*p.m01+t.w*p.m11;
    value=Mix(value,t,a);filled=filled*(1-a)+a;
  }
  if(filled<.999f){atomicExch(invalid,1u);return;}
  SurfaceTexel* out=job.out;
  float norm=fmaxf(1.f,hypotf(value.z,value.w)/.999f);
  if(macro.data){const Coord& cx=coords[size_t(blockIdx.z)*2*side+col];const Coord& cy=coords[(size_t(blockIdx.z)*2+1)*side+row];
    double fx=cx.f,fy=cy.f;int2 xi=cx.m,yi=cy.m;
    double sum=macro.data[size_t(yi.x)*macro.width+xi.x]*(1-fx)*(1-fy)+macro.data[size_t(yi.x)*macro.width+xi.y]*fx*(1-fy)+
               macro.data[size_t(yi.y)*macro.width+xi.x]*(1-fx)*fy+macro.data[size_t(yi.y)*macro.width+xi.y]*fx*fy;
    value.x*=float(sum/macro.scale);}
  out[size_t(row)*side+col]={static_cast<uint16_t>(fminf(65535.f,fmaxf(0.f,value.x*65535+.5f))),
    static_cast<uint8_t>(fminf(255.f,fmaxf(0.f,value.y*255+.5f))),0,
    static_cast<int16_t>(fminf(32767.f,fmaxf(-32767.f,value.z/norm*32767.f))),
    static_cast<int16_t>(fminf(32767.f,fmaxf(-32767.f,value.w/norm*32767.f)))};
}
}
struct CudaSurfaceRecipe::Impl {
  const SurfaceRecipe& recipe;std::vector<void*> allocations;
  RecipeSource* sources=nullptr;RecipePatch* patches=nullptr;unsigned char* alpha=nullptr;unsigned* invalid=nullptr;
  Job* jobs=nullptr;Coord* coords=nullptr;RecipeMacro macro;
  // Per-batch patch slots and their axes grow with the largest batch seen.
  Slot* slots=nullptr;Axis* axes=nullptr;size_t slot_capacity=0;
  size_t bytes=0;
  explicit Impl(const SurfaceRecipe& r):recipe(r){}
  void* Alloc(size_t n,const void* data=nullptr){void* p=nullptr;Check(cudaMalloc(&p,n));allocations.push_back(p);bytes+=n;
    if(data){for(size_t offset=0;offset<n;offset+=16u<<20){auto size=std::min(n-offset,size_t(16u<<20));Check(cudaMemcpy(static_cast<char*>(p)+offset,static_cast<const char*>(data)+offset,size,cudaMemcpyHostToDevice));}}return p;}
  void Reserve(size_t n,int side){
    if(n<=slot_capacity)return;
    if(slots){cudaFree(slots);cudaFree(axes);bytes-=slot_capacity*(sizeof(Slot)+2*side*sizeof(Axis));}
    slot_capacity=std::max(n,2*slot_capacity);void* p=nullptr;
    Check(cudaMalloc(&p,slot_capacity*sizeof(Slot)));slots=static_cast<Slot*>(p);
    Check(cudaMalloc(&p,slot_capacity*2*side*sizeof(Axis)));axes=static_cast<Axis*>(p);bytes+=slot_capacity*(sizeof(Slot)+2*side*sizeof(Axis));
  }
  ~Impl(){for(auto p:allocations)cudaFree(p);cudaFree(slots);cudaFree(axes);}
};
CudaSurfaceRecipe::CudaSurfaceRecipe(const SurfaceRecipe& r):impl_(std::make_unique<Impl>(r)){
  auto& s=*impl_;std::vector<RecipeSource> sources=r.sources;
  for(auto& source:sources)source.data=static_cast<const SurfaceTexel*>(s.Alloc(size_t(source.width)*source.height*sizeof(SurfaceTexel),source.data));
  s.sources=static_cast<RecipeSource*>(s.Alloc(sources.size()*sizeof(RecipeSource),sources.data()));
  s.patches=static_cast<RecipePatch*>(s.Alloc(r.patches.size()*sizeof(RecipePatch),r.patches.data()));
  s.alpha=static_cast<unsigned char*>(s.Alloc(r.alpha_bytes,r.alpha));
  s.jobs=static_cast<Job*>(s.Alloc(kMaxJobs*sizeof(Job)));
  s.coords=static_cast<Coord*>(s.Alloc(kMaxJobs*2*size_t(r.grid.core+2*r.grid.gutter)*sizeof(Coord)));
  s.macro=r.macro;if(r.macro.data)s.macro.data=static_cast<const uint16_t*>(s.Alloc(size_t(r.macro.width)*r.macro.height*2,r.macro.data));s.invalid=static_cast<unsigned*>(s.Alloc(sizeof(unsigned)));
}
CudaSurfaceRecipe::~CudaSurfaceRecipe()=default;
size_t CudaSurfaceRecipe::Bytes()const{return impl_->bytes;}
void CudaSurfaceRecipe::Generate(unsigned tile,SurfaceTexel* output,void* handle){Generate({{tile,output}},handle);}
void CudaSurfaceRecipe::Generate(const std::vector<std::pair<unsigned,SurfaceTexel*>>& all,void* handle){
  auto& s=*impl_;auto stream=static_cast<cudaStream_t>(handle);int side=s.recipe.grid.core+2*s.recipe.grid.gutter;
  for(size_t begin=0;begin<all.size();begin+=kMaxJobs){
    std::vector<Job> jobs;std::vector<Slot> slots;
    for(size_t i=begin;i<std::min(all.size(),begin+kMaxJobs);++i){auto selected=s.recipe.Select(all[i].first);
      jobs.push_back({all[i].first,int(slots.size()),int(selected.size()),all[i].second});
      for(int id:selected)slots.push_back({id,int(jobs.size()-1)});}
    if(slots.size()>65535)throw std::runtime_error("CUDA surface recipe: too many patches in one batch");
    s.Reserve(slots.size(),side);unsigned lines=(side+kLine-1)/kLine;
    Check(cudaMemcpyAsync(s.slots,slots.data(),slots.size()*sizeof(Slot),cudaMemcpyHostToDevice,stream));
    Check(cudaMemcpyAsync(s.jobs,jobs.data(),jobs.size()*sizeof(Job),cudaMemcpyHostToDevice,stream));
    Check(cudaMemsetAsync(s.invalid,0,sizeof(unsigned),stream));
    Coords<<<dim3(lines,2,jobs.size()),kLine,0,stream>>>(s.recipe.grid,s.jobs,s.macro,s.coords);
    Axes<<<dim3(lines,2,slots.size()),kLine,0,stream>>>(s.recipe.grid,s.slots,s.sources,s.patches,s.coords,s.axes);
    GenerateTile<<<dim3((side+kBlock-1)/kBlock,(side+kBlock-1)/kBlock,jobs.size()),dim3(kBlock,kBlock),0,stream>>>(s.recipe.grid,s.jobs,s.slots,s.sources,s.patches,s.alpha,s.coords,s.axes,s.macro,s.invalid);
    Check(cudaGetLastError());unsigned invalid=0;Check(cudaMemcpyAsync(&invalid,s.invalid,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));Check(cudaStreamSynchronize(stream));
    if(invalid)throw std::runtime_error("CUDA surface recipe: unfilled texels");
  }
}
}
