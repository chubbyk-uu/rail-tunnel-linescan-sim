#include "ssb_core/stage_b_assets.hpp"
#include "ssb_core/sha256.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ssb {
namespace {
constexpr double pi=3.14159265358979323846;
void Need(bool yes,const std::string& message) { if(!yes) throw std::runtime_error("Stage B: "+message); }
nlohmann::json ReadJson(const std::filesystem::path& p) {
  std::ifstream in(p); Need(bool(in),"cannot read "+p.string()); return nlohmann::json::parse(in);
}
std::filesystem::path Resolve(const std::filesystem::path& base,const std::string& name) {
  return std::filesystem::weakly_canonical(base/name);
}
template<class T> std::vector<T> ReadBinary(const std::filesystem::path& path,size_t bytes,const std::string& hash) {
  Need(std::filesystem::file_size(path)==bytes,"binary size mismatch: "+path.string());
  Need(Sha256File(path)==hash,"binary hash mismatch: "+path.string());
  std::vector<T> data(bytes/sizeof(T));std::ifstream in(path,std::ios::binary);
  in.read(reinterpret_cast<char*>(data.data()),bytes);Need(bool(in),"short binary read");return data;
}
Vec3 Sub(Vec3 a,Vec3 b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
Vec3 Cross(Vec3 a,Vec3 b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
double Dot(Vec3 a,Vec3 b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
}
StageBAssets::StageBAssets(const Config& c) {
  path=c.optical_scene;scene=ReadJson(path);scene_hash=Sha256File(path);
  Need(scene.at("schema")=="ssb.optical_scene.v1","unsupported optical scene");
  auto verified=[&](const nlohmann::json& entry) {
    auto p=Resolve(path.parent_path(),entry.at("file"));
    Need(Sha256File(p)==entry.at("sha256"),"scene asset identity mismatch");return p;
  };
  surface_path=verified(scene.at("surface"));surface=ReadJson(surface_path);surface_hash=Sha256File(surface_path);
  defect_path=verified(scene.at("defects"));defects=ReadJson(defect_path);defect_hash=Sha256File(defect_path);
  const bool runtime_recipe=surface.at("schema")=="ssb.surface_runtime.v1";
  Need((runtime_recipe || surface.at("schema")=="ssb.surface_tiles.v1") && defects.at("schema")=="ssb.defect_layout.v1","asset schema");
  Need(surface.at("texel_format")=="mono16_rough8_reserved8_nx16_nq16_le","surface texel format");
  for(const char* k:{"radius_m","axis_z_m","x_min_m","x_max_m"}) {
    double expected=k==std::string("radius_m")?c.tunnel_radius_m:k==std::string("axis_z_m")?c.tunnel_axis_z_m:
                    k==std::string("x_min_m")?c.tunnel_x_min_m:c.tunnel_x_max_m;
    Need(std::abs(surface.at("tunnel").at(k).get<double>()-expected)<1e-9,"surface/config tunnel mismatch");
  }
  core=surface.at("core_pixels");gutter=surface.at("gutter_pixels");side=core+2*gutter;
  nx=surface.at("tiles_xq")[0];nq=surface.at("tiles_xq")[1];
  pixel_x=surface.at("pixels_xq")[0];pixel_q=surface.at("pixels_xq")[1];
  x0=surface.at("origin_xq_m")[0];q0=surface.at("origin_xq_m")[1];
  dx=surface.at("texel_xq_m")[0];dq=surface.at("texel_xq_m")[1];period=surface.at("period_q_m");
  Need(core>0 && core<=2048 && gutter>=1 && gutter<=16 && nx>0 && nq>0 && nx*nq<=1000000,
       "surface dimensions outside limits");
  Need(std::isfinite(dx)&&std::isfinite(dq)&&dx>0&&dq>0&&std::abs(pixel_q*dq-period)<1e-8,"invalid texel scale");
  Need(std::abs(x0-c.tunnel_x_min_m)<1e-9 && std::abs(q0+pi*c.tunnel_radius_m)<1e-9 &&
       std::abs(period-2*pi*c.tunnel_radius_m)<1e-9 &&
       pixel_x>(nx-1)*core && pixel_x<=nx*core && pixel_q>(nq-1)*core && pixel_q<=nq*core &&
       pixel_x*dx>=c.tunnel_x_max_m-x0-1e-9 && pixel_x*dx<=c.tunnel_x_max_m-x0+dx+1e-9,
       "surface mapping domain mismatch");
  if(runtime_recipe) recipe=std::make_unique<SurfaceRecipe>(surface_path,surface);
  else {
    Need(surface.at("tiles").size()==size_t(nx)*nq,"incomplete tile manifest");
    for(unsigned i=0;i<nx*nq;++i) Need(surface.at("tiles")[i].at("ix")==i%nx && surface.at("tiles")[i].at("iq")==i/nx,"tile order");
  }
  gpu_budget=surface.at("resources").at("gpu_texture_budget_bytes");cpu_budget=surface.at("resources").at("cpu_texture_cache_bytes");
  if(scene.contains("texture_budgets")) {
    gpu_budget=scene.at("texture_budgets").at("gpu_bytes");cpu_budget=scene.at("texture_budgets").at("cpu_bytes");
  }
  tile_bytes=size_t(side)*side*sizeof(SurfaceTexel);cache_slots=gpu_budget/tile_bytes;
  Need(cache_slots>0 && cache_slots<=1024 && cpu_budget>=tile_bytes,"tile cache budget");
  auto sampling=scene.at("sampling");area_samples=sampling.at("area_axis_samples");time_samples=sampling.at("time_samples");
  adaptive_area=sampling.value("adaptive_area",false);
  const auto pattern=sampling.value("area_pattern",std::string("grid"));
  Need(pattern=="grid"||pattern=="rooks","area pattern");
  if(pattern=="rooks") {
    // Generator coprime with N maximising the minimum toroidal point distance (first on ties).
    double best=-1;
    for(unsigned g=1;g<std::max(2u,area_samples);++g) {
      if(std::gcd(g,area_samples)!=1)continue;double d=1e9;
      for(unsigned k=1;k<area_samples;++k){unsigned y=k*g%area_samples;
        d=std::min(d,std::hypot(double(std::min(k,area_samples-k)),double(std::min(y,area_samples-y))));}
      if(d>best){best=d;area_rooks=g;}
    }
  }
  integrated_cracks=sampling.value("integrated_cracks",false);
  texture_footprint_samples=sampling.value("texture_footprint_samples",1u);
  texture_prefilter=sampling.value("texture_prefilter",false);
  Need(texture_footprint_samples>=1&&texture_footprint_samples<=8,"texture footprint samples");
  convex_panel_visibility=scene.value("convex_panel_visibility",false);
  light_samples=scene.at("lamp").at("samples");
  Need(area_samples>=1&&area_samples<=16 && time_samples>=1&&time_samples<=16 && light_samples>=1&&light_samples<=9,"sample limits");
  Need(!integrated_cracks || time_samples==3,"integrated cracks require three exposure frames");
  auto lamp=scene.at("lamp");light_enabled=lamp.at("enabled");shadows=lamp.at("shadows");
  response_gain=scene.at("response_gain");lamp_length=lamp.at("length_m");footprint_x=lamp.at("footprint_m")[0];
  footprint_q=lamp.at("footprint_m")[1];lamp_tangential=lamp.at("offset_tangential_m");lamp_radial=lamp.at("offset_radial_m");
  Need(std::isfinite(response_gain)&&response_gain>0&&lamp_length>0&&footprint_x>0&&footprint_q>0,"lamp/response parameters");
  lamp_axial=lamp.value("offset_axial_m",0.);lamp_width=lamp.value("width_m",0.);
  Need(std::isfinite(lamp_axial)&&std::isfinite(lamp_width)&&lamp_width>=0 &&
       std::isfinite(lamp_tangential)&&std::isfinite(lamp_radial),"nonfinite lamp geometry");
  Need(lamp_width==0 || light_samples==4,"rectangular COB requires four quadrature points");
  max_radius=c.tunnel_radius_m;
  tunnel_radius=surface.at("tunnel").at("radius_m");tunnel_axis_z=surface.at("tunnel").at("axis_z_m");
  for(const auto& mesh:scene.at("meshes")) {
    auto p=verified(mesh);std::ifstream in(p);std::string line;
    const unsigned base=vertices.size();const unsigned material=mesh.at("material");
    while(std::getline(in,line)) {
      std::istringstream row(line);std::string kind;row>>kind;
      if(kind=="v") {
        OpticalVertex v;Need(bool(row>>v.x>>v.y>>v.z),"invalid OBJ vertex");
        Need(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z),"nonfinite vertex");
        vertices.push_back(v);max_radius=std::max(max_radius,std::hypot(double(v.y),double(v.z)-c.tunnel_axis_z_m));
      } else if(kind=="f") {
        std::string ia,ib,ic;Need(bool(row>>ia>>ib>>ic),"only triangle OBJ indices supported");
        auto index=[](const std::string& token){return unsigned(std::stoul(token.substr(0,token.find('/'))));};
        unsigned a=index(ia),b=index(ib),d=index(ic);
        Need(a>0&&b>0&&d>0&&std::max({a,b,d})<=vertices.size()-base,"invalid OBJ indices");
        triangles.push_back({base+a-1,base+b-1,base+d-1});face_material.push_back(material);
      }
      Need(vertices.size()<=1600000&&triangles.size()<=800000,"optical geometry exceeds budget");
    }
  }
  Need(!triangles.empty() && max_radius<c.tunnel_radius_m+.1,"empty/invalid optical geometry");
  const bool has_filler_faces=std::find(face_material.begin(),face_material.end(),2u)!=face_material.end();
  Need(std::all_of(face_material.begin(),face_material.end(),[](unsigned m){return m<=3;}),"unknown optical material");
  if(scene.contains("crack_optics")) {
    auto k=scene.at("crack_optics");crack_interior=k.at("interior_ratio");crack_interior_variation=k.at("interior_variation");
    crack_edge_band=k.at("edge_band_m");crack_edge_darkening=k.at("edge_darkening");
    Need(crack_interior>=0&&crack_interior<1&&crack_interior_variation>=0&&crack_interior_variation<=1&&
         crack_edge_band>=0&&crack_edge_band<.002&&crack_edge_darkening>=0&&crack_edge_darkening<1,"crack optics parameters");
  }
  if(scene.contains("joint_concrete")) {
    auto j=scene.at("joint_concrete");groove_albedo=j.at("albedo");groove_detail_contrast=j.at("detail_contrast");
    gap_albedo=j.value("gap_albedo",.02);
    Need(groove_albedo>0&&groove_albedo<1&&groove_detail_contrast>=0&&groove_detail_contrast<=1&&gap_albedo>=0&&gap_albedo<1,"joint concrete parameters");
  }
  if(scene.contains("filler")) {
    auto meta_path=verified(scene.at("filler"));auto meta=ReadJson(meta_path);filler_hash=Sha256File(meta_path);
    Need(meta.at("schema")=="ssb.joint_filler.v1","filler schema");
    filler_width=meta.at("width");filler_height=meta.at("height");filler_pitch=meta.at("pitch_m");filler_scale=meta.at("scale");
    filler_mean=meta.at("mean_albedo");filler_roughness=meta.value("roughness",.9);
    Need(filler_width>1&&filler_height>1&&filler_width<=16384&&filler_height<=16384&&filler_pitch>0&&filler_scale>0&&
         filler_mean>0&&filler_mean<1,"filler dimensions");
    auto bin=Resolve(meta_path.parent_path(),meta.at("file"));
    Need(std::filesystem::file_size(bin)==size_t(filler_width)*filler_height*2 && Sha256File(bin)==meta.at("sha256"),"filler payload identity");
    filler.resize(size_t(filler_width)*filler_height);std::ifstream(bin,std::ios::binary).read(reinterpret_cast<char*>(filler.data()),filler.size()*2);
  }
  Need(!has_filler_faces || !filler.empty(),"filler faces require a scene filler texture");
  if(integrated_cracks) {
    // OBJ quads duplicate vertices. Match geometric edges, including periodic
    // boundaries; internal diagonals and <=1 degree wall facets are smooth.
    using Point=std::array<float,3>;using Edge=std::array<Point,2>;
    std::map<Edge,std::vector<std::pair<unsigned,unsigned>>> edges;
    std::vector<Vec3> normals;
    for(unsigned i=0;i<triangles.size();++i) {
      auto t=triangles[i];unsigned ids[3]={t.a,t.b,t.c};
      auto p=[&](unsigned id){auto v=vertices[id];return Point{v.x,v.y,v.z};};
      auto a=p(t.a),b=p(t.b),d=p(t.c);
      Vec3 av{a[0],a[1],a[2]},bv{b[0],b[1],b[2]},dv{d[0],d[1],d[2]};
      Vec3 n=Cross(Sub(bv,av),Sub(dv,av));
      double length=std::sqrt(Dot(n,n));for(double& v:n)v/=length;normals.push_back(n);
      for(unsigned e=0;e<3;++e) {Edge key={p(ids[e]),p(ids[(e+1)%3])};if(key[1]<key[0])std::swap(key[0],key[1]);edges[key].push_back({i,e});}
    }
    critical_edges.assign(triangles.size(),0);
    for(const auto& item:edges) {
      const auto& neighbors=item.second;bool critical=neighbors.size()!=2;
      if(!critical) {auto a=neighbors[0].first,b=neighbors[1].first;
        critical=face_material[a]!=face_material[b] || Dot(normals[a],normals[b])<std::cos(pi/180.);}
      if(critical)for(auto edge:neighbors)critical_edges[edge.first]|=1u<<edge.second;
    }
  }
  if(adaptive_area && (!surface.contains("adaptive_crack_guard_m") ||
     surface.value("adaptive_crack_guard_m",0.)<.002 || surface.value("adaptive_defects_sha256",std::string())!=defect_hash))
    adaptive_area=false;  // No acceleration mask: retain full area sampling.
  auto grid=defects.at("grid");crack_x0=grid.at("origin_xq_m")[0];crack_q0=grid.at("origin_xq_m")[1];
  crack_cell=grid.at("cell_m");crack_nx=grid.at("cells_xq")[0];crack_nq=grid.at("cells_xq")[1];
  const size_t segment_count=grid.at("segments"),index_count=grid.at("index_entries");
  Need(crack_cell>0&&crack_nx>0&&crack_nq>0&&segment_count*sizeof(CrackSegment)+(size_t(crack_nx)*crack_nq+1)*4+index_count*4<=(32u<<20),"crack grid budget");
  auto file=[&](const char* name,size_t bytes,auto tag) {
    const auto entry=defects.at("files").at(name);using T=decltype(tag);
    return ReadBinary<T>(Resolve(defect_path.parent_path(),entry.at("file")),bytes,entry.at("sha256"));
  };
  segments=file("segments.bin",segment_count*sizeof(CrackSegment),CrackSegment{});
  offsets=file("offsets.bin",(size_t(crack_nx)*crack_nq+1)*4,unsigned{});
  indices=file("indices.bin",index_count*4,unsigned{});
  Need(offsets.front()==0&&offsets.back()==indices.size()&&std::is_sorted(offsets.begin(),offsets.end()),"invalid crack offsets");
  for(auto index:indices) Need(index<segments.size(),"invalid crack segment index");
  for(auto s:segments) Need(std::isfinite(s.x0)&&std::isfinite(s.q0)&&std::isfinite(s.x1)&&std::isfinite(s.q1)&&
                            std::isfinite(s.r0)&&std::isfinite(s.r1)&&s.r0>=0&&s.r1>=0&&s.r0<=.0003001&&s.r1<=.0003001,"invalid crack segment");
}
std::vector<SurfaceTexel> StageBAssets::ReadTile(unsigned index) const {
  if(index>=nx*nq)throw std::runtime_error("Stage B: tile index out of range");
  if(recipe)return recipe->Generate(index);
  const auto entry=surface.at("tiles").at(index);
  return ReadBinary<SurfaceTexel>(Resolve(surface_path.parent_path(),entry.at("file")),tile_bytes,entry.at("sha256"));
}
StageBAssets::FootprintWindow StageBAssets::Window(const HeadPose& h,double pixel_step,double max_tan) const {
  // Bound every ray of the pixel area. For O+a*L+b*S the transverse norm has a
  // conservative lower bound. This also includes groove depth and off-axis mounts.
  double t=max_tan+pixel_step;
  double a=h.line[1]*h.line[1]+h.line[2]*h.line[2];
  double b=h.optical[1]*h.line[1]+h.optical[2]*h.line[2];
  double u=a>0?std::clamp(-b/a,-t,t):0;
  double min_trans=std::hypot(h.optical[1]+u*h.line[1],h.optical[2]+u*h.line[2])-pixel_step;
  Need(min_trans>.01,"line points outside supported cache geometry");
  double offset=std::hypot(h.origin[1],h.origin[2]-tunnel_axis_z);
  double travel=(max_radius+offset)/min_trans;
  double half_x=travel*(std::abs(h.optical[0])+t*std::abs(h.line[0])+pixel_step*std::abs(h.scan[0]))+2*dx;
  double theta=std::atan2(h.optical[1],h.optical[2]);
  // Angular projection of line direction plus transverse offset, expanded for
  // normal/area sampling. This is conservative even when line twist is nonzero.
  double transverse_base=std::hypot(h.optical[1],h.optical[2]);
  double angle=std::asin(std::clamp((t*std::sqrt(a)+pixel_step)/transverse_base,0.,1.))+
               std::asin(std::clamp(offset/tunnel_radius,0.,1.));
  double half_q=tunnel_radius*angle+2*dq;
  int ix0=std::max(0,int(std::floor((h.origin[0]-half_x-x0)/(core*dx))));
  int ix1=std::min(int(nx)-1,int(std::floor((h.origin[0]+half_x-x0)/(core*dx))));
  return {ix0,ix1,tunnel_radius*theta,half_q};
}
std::set<unsigned> StageBAssets::Footprint(const HeadPose& h,double pixel_step,double max_tan) const {
  const auto win=Window(h,pixel_step,max_tan);
  const int ix0=win.ix0,ix1=win.ix1;const double centre=win.centre_q,half_q=win.half_q;
  std::set<unsigned> result;
  // Periodic q tiles have a truncated final core: use physical period, never nq*core*dq.
  // Tile iq covers [q0+iq*w, min(q0+period, q0+(iq+1)*w)]; collect the tiles overlapping the
  // window shifted by -1/0/+1 periods (same inclusive test as a scan over all tiles).
  const double w=core*dq;
  for(int k=-1;k<=1;++k) {
    double lo=centre-half_q-k*period-q0, hi=centre+half_q-k*period-q0;
    if(hi<0||lo>period) continue;
    int first=std::max(0,int(std::ceil(lo/w))-1), last=std::min(int(nq)-1,int(std::floor(hi/w)));
    for(int iq=first;iq<=last;++iq) {
      double tlo=iq*w,thi=std::min(period,tlo+w);
      if(thi>=lo&&tlo<=hi) for(int ix=ix0;ix<=ix1;++ix) result.insert(iq*nx+ix);
    }
  }
  return result;
}
bool StageBAssets::CpuHit(const HeadPose& h,double tangent,double scan_tangent,double* x,double* q) const {
  Vec3 direction;for(int k=0;k<3;++k) direction[k]=h.optical[k]+tangent*h.line[k]+scan_tangent*h.scan[k];
  double closest=1e30;
  for(auto triangle:triangles) {
    auto point=[&](unsigned index){auto v=vertices[index];return Vec3{v.x,v.y,v.z};};
    Vec3 a=point(triangle.a),e1=Sub(point(triangle.b),a),e2=Sub(point(triangle.c),a);
    Vec3 p=Cross(direction,e2);double det=Dot(e1,p);if(std::abs(det)<1e-12) continue;
    Vec3 t=Sub(h.origin,a);double u=Dot(t,p)/det;if(u<0||u>1) continue;
    Vec3 v=Cross(t,e1);double w=Dot(direction,v)/det;if(w<0||u+w>1) continue;
    double distance=Dot(e2,v)/det;if(distance>0&&distance<closest) closest=distance;
  }
  if(closest==1e30) return false;
  *x=h.origin[0]+closest*direction[0];
  *q=surface.at("tunnel").at("radius_m").get<double>()*std::atan2(h.origin[1]+closest*direction[1],
       h.origin[2]+closest*direction[2]-surface.at("tunnel").at("axis_z_m").get<double>());
  return true;
}
}
