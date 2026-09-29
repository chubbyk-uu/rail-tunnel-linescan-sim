#include "ssb_core/surface_recipe.hpp"
#include "ssb_core/sha256.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <stdexcept>

namespace ssb {
namespace {
void Need(bool b,const char* message){if(!b)throw std::runtime_error(std::string("surface recipe: ")+message);}
double Clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
void Coordinates(const RecipeGrid& g,unsigned tile,int col,int row,double& gx,double& gq){
  double xp=Clamp(double(tile%g.nx*g.core)+col-g.gutter+.5,.5,g.pixel_x-.5);
  double qp=std::fmod(double(tile/g.nx*g.core)+row-g.gutter+.5,g.pixel_q);
  if(qp<0)qp+=g.pixel_q;
  gx=(g.x0+xp*g.dx-g.guide_x0)/g.guide_step;
  gq=(g.q0+qp*g.dq-g.guide_q0)/g.guide_step;
}
template<class F> double Bilinear(double x,double y,int side,F get){
  // Fixed 1/32 interpolation grid agrees with the archived OpenCV quilt convention.
  long u=long(std::floor(x*32+.5)),v=long(std::floor(y*32+.5));
  int x0=int(std::floor(double(u)/32)),y0=int(std::floor(double(v)/32));
  int fx=int(u-x0*32),fy=int(v-y0*32);double sum=0;
  for(int j=0;j<2;++j)for(int i=0;i<2;++i)
    sum+=get(std::clamp(x0+i,0,side-1),std::clamp(y0+j,0,side-1))*(i?fx:32-fx)*(j?fy:32-fy);
  return sum/1024;
}
}
struct SurfaceRecipe::Mapping {
  void* data=MAP_FAILED;size_t bytes=0;
  Mapping(const std::filesystem::path& p,size_t n,const std::string& hash):bytes(n){
    Need(std::filesystem::file_size(p)==n && Sha256File(p)==hash,"payload size/hash mismatch");
    int fd=open(p.c_str(),O_RDONLY);Need(fd>=0,"payload open failed");
    data=mmap(nullptr,n,PROT_READ,MAP_PRIVATE,fd,0);close(fd);Need(data!=MAP_FAILED,"mmap failed");
  }
  ~Mapping(){if(data!=MAP_FAILED)munmap(data,bytes);}
};
SurfaceRecipe::~SurfaceRecipe()=default;
SurfaceRecipe::SurfaceRecipe(const std::filesystem::path& path,const nlohmann::json& s){
  auto resolve=[&](const nlohmann::json& e,const std::filesystem::path& base){
    auto name=e.at("file").get<std::string>();Need(!std::filesystem::path(name).is_absolute(),"absolute payload path");
    auto p=std::filesystem::weakly_canonical(base/name),root=std::filesystem::weakly_canonical(base);
    auto rel=p.lexically_relative(root);Need(!rel.empty()&&*rel.begin()!="..","payload escapes recipe directory");return p;
  };
  auto rp=resolve(s.at("recipe"),path.parent_path());
  Need(Sha256File(rp)==s.at("recipe").at("sha256"),"recipe hash mismatch");
  nlohmann::json r;std::ifstream(rp)>>r;Need(r.at("schema")=="ssb.surface_recipe.v1","schema");
  Need(r.at("interpolation")=="bilinear_fixed32_v1","interpolation");
  grid={s.at("core_pixels"),s.at("gutter_pixels"),s.at("tiles_xq")[0],s.at("pixels_xq")[0],s.at("pixels_xq")[1],
    s.at("origin_xq_m")[0],s.at("origin_xq_m")[1],s.at("texel_xq_m")[0],s.at("texel_xq_m")[1],s.at("period_q_m"),
    r.at("origin_xq_m")[0],r.at("origin_xq_m")[1],r.at("guide_texel_m"),r.at("patch_pixels")};
  Need(grid.core>0&&grid.core<=2048&&grid.gutter>=1&&grid.gutter<=16&&grid.nx>0&&grid.pixel_x>0&&grid.pixel_q>0&&
       std::isfinite(grid.dx)&&std::isfinite(grid.dq)&&grid.dx>0&&grid.dq>0,"grid dimensions");
  Need(std::isfinite(grid.guide_x0)&&std::isfinite(grid.guide_q0)&&std::isfinite(grid.guide_step)&&grid.guide_step>0&&grid.patch_side>=2&&grid.patch_side<=1024,"guide dimensions");
  for(const auto& e:r.at("sources")){
    int side=e.at("side");double width=e.at("source_width_m");
    Need(side>1&&side<=16384&&std::isfinite(width)&&width>0,"source dimensions");
    size_t n=size_t(side)*side*sizeof(SurfaceTexel);
    Need(source_bytes+n<=s.at("resources").value("gpu_source_budget_bytes",size_t(4ull<<30)),"source budget exceeded");
    maps_.push_back(std::make_unique<Mapping>(resolve(e,rp.parent_path()),n,e.at("sha256")));
    sources.push_back({static_cast<const SurfaceTexel*>(maps_.back()->data),side,width/side});source_bytes+=n;
  }
  Need(!sources.empty()&&sources.size()<=8,"source count");
  for(const auto& p:r.at("placements")){
    auto m=p.at("source_matrix");RecipePatch item={p.at("left"),p.at("top"),p.at("material"),m[0][0],m[0][1],m[1][0],m[1][1],p.at("source_offset_m")[0],p.at("source_offset_m")[1]};
    Need(item.source>=0&&item.source<int(sources.size())&&std::isfinite(item.ox)&&std::isfinite(item.oq),"patch source/offset");
    Need(std::abs(item.m00)<=1&&std::abs(item.m01)<=1&&std::abs(item.m10)<=1&&std::abs(item.m11)<=1&&item.m00*item.m00+item.m01*item.m01==1&&item.m10*item.m10+item.m11*item.m11==1&&item.m00*item.m10+item.m01*item.m11==0,"orientation must be orthogonal");
    patches.push_back(item);
  }
  Need(!patches.empty()&&patches.size()<=10000,"patch count");
  auto e=r.at("alpha");alpha_bytes=patches.size()*size_t(grid.patch_side)*grid.patch_side;
  maps_.push_back(std::make_unique<Mapping>(resolve(e,rp.parent_path()),alpha_bytes,e.at("sha256")));
  alpha=static_cast<const unsigned char*>(maps_.back()->data);
}
std::vector<int> SurfaceRecipe::Select(unsigned tile) const {
  const auto& g=grid;int side=g.core+2*g.gutter;
  Need(tile<size_t(g.nx)*((g.pixel_q+g.core-1)/g.core),"tile index outside domain");
  double xmin=1e30,xmax=-1e30,qmin=1e30,qmax=-1e30;
  for(int i=0;i<side;++i){double x,q;Coordinates(g,tile,i,i,x,q);xmin=std::min(xmin,x);xmax=std::max(xmax,x);qmin=std::min(qmin,q);qmax=std::max(qmax,q);}
  std::vector<int> ids;
  for(size_t i=0;i<patches.size();++i){auto p=patches[i];if(xmax>=p.left&&xmin<p.left+g.patch_side&&qmax>=p.top&&qmin<p.top+g.patch_side)ids.push_back(i);}
  Need(!ids.empty(),"tile outside layout");return ids;
}
std::vector<SurfaceTexel> SurfaceRecipe::Generate(unsigned tile) const {
  auto ids=Select(tile);const auto& g=grid;int side=g.core+2*g.gutter;
  std::vector<SurfaceTexel> out(size_t(side)*side);
  for(int row=0;row<side;++row)for(int col=0;col<side;++col){
    double gx,gq;Coordinates(g,tile,col,row,gx,gq);double value[4]={},filled=0;
    for(auto id:ids){auto p=patches[id];double px=gx-p.left,py=gq-p.top;
      if(px<0||py<0||px>=g.patch_side||py>=g.patch_side)continue;
      auto mask=alpha+size_t(id)*g.patch_side*g.patch_side;
      double a=Bilinear(px-.5,py-.5,g.patch_side,[&](int x,int y){return double(mask[y*g.patch_side+x]);})/255;
      auto src=sources[p.source];double u=(p.m00*px*g.guide_step+p.m01*py*g.guide_step+p.ox)/src.native-.5;
      double v=(p.m10*px*g.guide_step+p.m11*py*g.guide_step+p.oq)/src.native-.5;
      double raw[4];for(int c=0;c<4;++c)raw[c]=Bilinear(u,v,src.side,[&](int x,int y){auto t=src.data[size_t(y)*src.side+x];return c==0?double(t.albedo)/65535:c==1?double(t.roughness)/255:c==2?double(t.nx)/32767:double(t.nq)/32767;});
      double z=raw[2];raw[2]=z*p.m00+raw[3]*p.m10;raw[3]=z*p.m01+raw[3]*p.m11;
      for(int c=0;c<4;++c)value[c]=value[c]*(1-a)+raw[c]*a;
      filled=filled*(1-a)+a;
    }
    Need(filled>=.999,"unfilled texel");double norm=std::max(1.,std::hypot(value[2],value[3])/.999);
    out[size_t(row)*side+col]={uint16_t(Clamp(value[0]*65535+.5,0,65535)),uint8_t(Clamp(value[1]*255+.5,0,255)),0,int16_t(Clamp(value[2]/norm*32767,-32767,32767)),int16_t(Clamp(value[3]/norm*32767,-32767,32767))};
  }return out;
}
}
