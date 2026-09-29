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
__device__ float4 Source(const RecipeSource& s,double x,double y){
  int u=int(floor(x*32+.5)),v=int(floor(y*32+.5)),x0=int(floor(double(u)/32)),y0=int(floor(double(v)/32));
  int fx=u-x0*32,fy=v-y0*32;int a=0,b=0,c=0,d=0;
  for(int j=0;j<2;++j)for(int i=0;i<2;++i){
    auto t=s.data[size_t(max(0,min(s.side-1,y0+j)))*s.side+max(0,min(s.side-1,x0+i))];
    int w=(i?fx:32-fx)*(j?fy:32-fy);a+=int(t.albedo)*w;b+=int(t.roughness)*w;c+=int(t.nx)*w;d+=int(t.nq)*w;
  }return make_float4(a/(1024.f*65535.f),b/(1024.f*255.f),c/(1024.f*32767.f),d/(1024.f*32767.f));
}
__device__ float Alpha(const unsigned char* mask,int side,double x,double y){
  int u=int(floor(x*32+.5)),v=int(floor(y*32+.5)),x0=int(floor(double(u)/32)),y0=int(floor(double(v)/32)),fx=u-x0*32,fy=v-y0*32,sum=0;
  for(int j=0;j<2;++j)for(int i=0;i<2;++i)sum+=mask[max(0,min(side-1,y0+j))*side+max(0,min(side-1,x0+i))]*(i?fx:32-fx)*(j?fy:32-fy);
  return sum/(1024.f*255.f);
}
__global__ void GenerateTile(RecipeGrid g,unsigned tile,const RecipeSource* sources,const RecipePatch* patches,
 const unsigned char* alpha,const int* ids,int count,SurfaceTexel* out,unsigned* invalid){
  int col=blockIdx.x*blockDim.x+threadIdx.x,row=blockIdx.y*blockDim.y+threadIdx.y,side=g.core+2*g.gutter;
  if(col>=side||row>=side)return;
  double xp=fmax(.5,fmin(g.pixel_x-.5,double(tile%g.nx*g.core)+col-g.gutter+.5));
  double qp=fmod(double(tile/g.nx*g.core)+row-g.gutter+.5,double(g.pixel_q));if(qp<0)qp+=g.pixel_q;
  double gx=(g.x0+xp*g.dx-g.guide_x0)/g.guide_step,gy=(g.q0+qp*g.dq-g.guide_q0)/g.guide_step;
  float4 value=make_float4(0,0,0,0);float filled=0;
  for(int k=0;k<count;++k){int id=ids[k];auto p=patches[id];double px=gx-p.left,py=gy-p.top;
    if(px<0||py<0||px>=g.patch_side||py>=g.patch_side)continue;
    float a=Alpha(alpha+size_t(id)*g.patch_side*g.patch_side,g.patch_side,px-.5,py-.5);
    auto src=sources[p.source];double u=(p.m00*px*g.guide_step+p.m01*py*g.guide_step+p.ox)/src.native-.5;
    double v=(p.m10*px*g.guide_step+p.m11*py*g.guide_step+p.oq)/src.native-.5;
    auto t=Source(src,u,v);float z=t.z;t.z=z*p.m00+t.w*p.m10;t.w=z*p.m01+t.w*p.m11;
    value=Mix(value,t,a);filled=filled*(1-a)+a;
  }
  if(filled<.999f){atomicExch(invalid,1u);return;}
  float norm=fmaxf(1.f,hypotf(value.z,value.w)/.999f);
  out[size_t(row)*side+col]={static_cast<uint16_t>(fminf(65535.f,fmaxf(0.f,value.x*65535+.5f))),
    static_cast<uint8_t>(fminf(255.f,fmaxf(0.f,value.y*255+.5f))),0,
    static_cast<int16_t>(fminf(32767.f,fmaxf(-32767.f,value.z/norm*32767.f))),
    static_cast<int16_t>(fminf(32767.f,fmaxf(-32767.f,value.w/norm*32767.f)))};
}
}
struct CudaSurfaceRecipe::Impl {
  const SurfaceRecipe& recipe;std::vector<void*> allocations;
  RecipeSource* sources=nullptr;RecipePatch* patches=nullptr;unsigned char* alpha=nullptr;int* ids=nullptr;unsigned* invalid=nullptr;
  size_t bytes=0;
  explicit Impl(const SurfaceRecipe& r):recipe(r){}
  void* Alloc(size_t n,const void* data=nullptr){void* p=nullptr;Check(cudaMalloc(&p,n));allocations.push_back(p);bytes+=n;
    if(data){for(size_t offset=0;offset<n;offset+=16u<<20){auto size=std::min(n-offset,size_t(16u<<20));Check(cudaMemcpy(static_cast<char*>(p)+offset,static_cast<const char*>(data)+offset,size,cudaMemcpyHostToDevice));}}return p;}
  ~Impl(){for(auto p:allocations)cudaFree(p);}
};
CudaSurfaceRecipe::CudaSurfaceRecipe(const SurfaceRecipe& r):impl_(std::make_unique<Impl>(r)){
  auto& s=*impl_;std::vector<RecipeSource> sources=r.sources;
  for(auto& source:sources)source.data=static_cast<const SurfaceTexel*>(s.Alloc(size_t(source.side)*source.side*sizeof(SurfaceTexel),source.data));
  s.sources=static_cast<RecipeSource*>(s.Alloc(sources.size()*sizeof(RecipeSource),sources.data()));
  s.patches=static_cast<RecipePatch*>(s.Alloc(r.patches.size()*sizeof(RecipePatch),r.patches.data()));
  s.alpha=static_cast<unsigned char*>(s.Alloc(r.alpha_bytes,r.alpha));
  s.ids=static_cast<int*>(s.Alloc(r.patches.size()*sizeof(int)));s.invalid=static_cast<unsigned*>(s.Alloc(sizeof(unsigned)));
}
CudaSurfaceRecipe::~CudaSurfaceRecipe()=default;
size_t CudaSurfaceRecipe::Bytes()const{return impl_->bytes;}
void CudaSurfaceRecipe::Generate(unsigned tile,SurfaceTexel* output,void* handle){
  auto& s=*impl_;auto stream=static_cast<cudaStream_t>(handle);auto selected=s.recipe.Select(tile);
  Check(cudaMemcpyAsync(s.ids,selected.data(),selected.size()*sizeof(int),cudaMemcpyHostToDevice,stream));
  Check(cudaMemsetAsync(s.invalid,0,sizeof(unsigned),stream));int side=s.recipe.grid.core+2*s.recipe.grid.gutter;
  GenerateTile<<<dim3((side+15)/16,(side+15)/16),dim3(16,16),0,stream>>>(s.recipe.grid,tile,s.sources,s.patches,s.alpha,s.ids,selected.size(),output,s.invalid);
  Check(cudaGetLastError());unsigned invalid=0;Check(cudaMemcpyAsync(&invalid,s.invalid,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));Check(cudaStreamSynchronize(stream));
  if(invalid)throw std::runtime_error("CUDA surface recipe: unfilled texels");
}
}
