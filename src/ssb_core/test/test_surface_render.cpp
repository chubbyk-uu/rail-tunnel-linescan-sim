#include <gtest/gtest.h>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <cstring>
#include <cuda_runtime.h>
#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/stage_b_assets.hpp"
#include "ssb_core/sha256.hpp"
#include "ssb_core/crack_integral.hpp"

namespace {
using namespace ssb;
constexpr double pi=3.14159265358979323846;
struct SurfaceFixture : ::testing::Test {
  Config c;
  std::filesystem::path root;
  nlohmann::json scene;
  void Json(const std::filesystem::path& file,const nlohmann::json& data) {std::ofstream(root/file)<<data.dump(2)<<"\n";}
  template<class T> void Binary(const std::string& file,const std::vector<T>& data) {
    std::ofstream out(root/file,std::ios::binary);out.write(reinterpret_cast<const char*>(data.data()),data.size()*sizeof(T));
  }
  nlohmann::json Entry(const std::string& name) {return {{"file",name},{"sha256",Sha256File(root/name)}};}
  void SetUp() override {
    root=std::filesystem::temp_directory_path()/("ssb_surface_test_"+std::to_string(getpid()));
    std::filesystem::remove_all(root);std::filesystem::create_directories(root);
    c=Config::Load(std::string(SSB_CONFIG_DIR)+"/stage_b.yaml");
    c.optical_scene=root/"scene.json";c.debug_column_stride=256;
    std::ofstream mesh(root/"wall.obj");mesh.precision(10);
    for(int i=0;i<720;++i) {
      double a=2*pi*i/720,b=2*pi*(i+1)/720;
      for(auto v:{Vec3{c.tunnel_x_min_m,2.75*std::sin(a),c.tunnel_axis_z_m+2.75*std::cos(a)},
                  Vec3{c.tunnel_x_min_m,2.75*std::sin(b),c.tunnel_axis_z_m+2.75*std::cos(b)},
                  Vec3{c.tunnel_x_max_m,2.75*std::sin(b),c.tunnel_axis_z_m+2.75*std::cos(b)},
                  Vec3{c.tunnel_x_max_m,2.75*std::sin(a),c.tunnel_axis_z_m+2.75*std::cos(a)}})
        mesh<<"v "<<v[0]<<" "<<(std::abs(v[1])<1e-14?0.:v[1])<<" "<<v[2]<<"\n";
      int start=i*4+1;mesh<<"f "<<start<<" "<<start+1<<" "<<start+2<<"\nf "<<start<<" "<<start+2<<" "<<start+3<<"\n";
    }
    mesh.close();
    const unsigned core=16,gutter=2,side=20,nx=3,nq=4;
    std::vector<SurfaceTexel> texels(side*side,{32768,0,0,0,0});
    nlohmann::json tiles=nlohmann::json::array();
    for(unsigned iq=0;iq<nq;++iq) for(unsigned ix=0;ix<nx;++ix) {
      for(unsigned row=0;row<side;++row) for(unsigned col=0;col<side;++col) {
        double x=-1.5+(int(ix*core+col)-int(gutter)+.5)*23./(core*nx);
        double q=-pi*2.75+(int(iq*core+row)-int(gutter)+.5)*2*pi*2.75/(core*nq);
        texels[row*side+col].reserved=std::abs(q-.5)<2*pi*2.75/(core*nq)+.003 && x>3-23./(core*nx)-.003 && x<13+23./(core*nx)+.003;
      }
      std::string file="tile_"+std::to_string(iq*nx+ix)+".bin";Binary(file,texels);
      auto entry=Entry(file);entry["ix"]=ix;entry["iq"]=iq;entry["bytes"]=texels.size()*sizeof(SurfaceTexel);tiles.push_back(entry);
    }
    Json("surface.json",{{"schema","ssb.surface_tiles.v1"},
      {"texel_format","mono16_rough8_reserved8_nx16_nq16_le"},
      {"tunnel",{{"radius_m",c.tunnel_radius_m},{"axis_z_m",c.tunnel_axis_z_m},{"x_min_m",c.tunnel_x_min_m},{"x_max_m",c.tunnel_x_max_m}}},
      {"core_pixels",core},{"gutter_pixels",gutter},{"tiles_xq",{nx,nq}},{"pixels_xq",{core*nx,core*nq}},
      {"origin_xq_m",{c.tunnel_x_min_m,-pi*2.75}},{"texel_xq_m",{23./(core*nx),2*pi*2.75/(core*nq)}},
      {"period_q_m",2*pi*2.75},{"tiles",tiles},
      {"resources",{{"gpu_texture_budget_bytes",side*side*8*4},{"cpu_texture_cache_bytes",side*side*8}}}});
    Binary("segments.bin",std::vector<CrackSegment>{{3,.5,13,.5,.0002,.0002}});
    const unsigned cx=46,cq=35;std::vector<unsigned> offsets(cx*cq+1),indices(cx*cq,0);
    for(unsigned i=0;i<offsets.size();++i) offsets[i]=i;
    Binary("offsets.bin",offsets);Binary("indices.bin",indices);
    Json("defects.json",{{"schema","ssb.defect_layout.v1"},
      {"grid",{{"origin_xq_m",{-1.5,-pi*2.75}},{"cell_m",.5},{"cells_xq",{cx,cq}},{"segments",1},{"index_entries",indices.size()}}},
      {"files",{{"segments.bin",Entry("segments.bin")},{"offsets.bin",Entry("offsets.bin")},{"indices.bin",Entry("indices.bin")}}}});
    nlohmann::json surface;std::ifstream(root/"surface.json")>>surface;
    surface["adaptive_crack_guard_m"]=.003;surface["adaptive_defects_sha256"]=Sha256File(root/"defects.json");
    Json("surface.json",surface);
    auto mesh_entry=Entry("wall.obj");mesh_entry["material"]=0;
    scene={{"schema","ssb.optical_scene.v1"},{"surface",Entry("surface.json")},{"defects",Entry("defects.json")},
      {"meshes",nlohmann::json::array({mesh_entry})},{"sampling",{{"area_axis_samples",2},{"time_samples",2}}},
      {"lamp",{{"enabled",false},{"shadows",false},{"samples",3},{"length_m",.3},{"footprint_m",{1.2,.12}},
               {"offset_tangential_m",.2},{"offset_radial_m",.025}}},{"response_gain",1.}};
    SaveScene();
  }
  void SaveScene(){Json("scene.json",scene);}
  void SetWidth(double width_m) {
    Binary("segments.bin",std::vector<CrackSegment>{{3,.5,13,.5,float(width_m/2),float(width_m/2)}});
    nlohmann::json defects;std::ifstream(root/"defects.json")>>defects;
    defects["files"]["segments.bin"]=Entry("segments.bin");Json("defects.json",defects);
    nlohmann::json surface;std::ifstream(root/"surface.json")>>surface;
    surface["adaptive_defects_sha256"]=Sha256File(root/"defects.json");Json("surface.json",surface);
    scene["defects"]=Entry("defects.json");scene["surface"]=Entry("surface.json");SaveScene();
  }
  void TearDown() override {std::filesystem::remove_all(root);}
};

TEST_F(SurfaceFixture, RuntimeRecipePreservesOrientationPeriodicGuttersAndRegeneration) {
  nlohmann::json surface;std::ifstream(root/"surface.json")>>surface;
  std::vector<SurfaceTexel> source(64*64);
  for(int q=0;q<64;++q)for(int x=0;x<64;++x)
    source[q*64+x]={uint16_t(1000+701*x+293*q),uint8_t(20+(x+q)%210),0,int16_t((x-32)*400),int16_t((q-32)*300)};
  Binary("native.bin",source);
  nlohmann::json placements=nlohmann::json::array();
  std::vector<unsigned char> alpha(8*8*8,32);std::fill(alpha.begin(),alpha.begin()+64,255);
  // All eight rotation/mirror transforms, blended with ordered masks.
  int m[8][4]={{1,0,0,1},{-1,0,0,1},{0,-1,1,0},{0,1,1,0},{-1,0,0,-1},{1,0,0,-1},{0,1,-1,0},{0,-1,-1,0}};
  for(int i=0;i<8;++i){auto a=m[i];placements.push_back({{"left",0},{"top",0},{"material",0},
    {"source_matrix",{{a[0],a[1]},{a[2],a[3]}}},{"source_offset_m",{(a[0]<0||a[1]<0)?32.:0.,(a[2]<0||a[3]<0)?32.:0.}}});}
  Binary("alpha.bin",alpha);
  auto src=Entry("native.bin");src["side"]=64;src["source_width_m"]=32.;
  Json("recipe.json",{{"schema","ssb.surface_recipe.v1"},{"interpolation","bilinear_fixed32_v1"},
    {"origin_xq_m",{-1.5,-pi*2.75}},{"guide_texel_m",4.},{"patch_pixels",8},
    {"sources",nlohmann::json::array({src})},{"placements",placements},{"alpha",Entry("alpha.bin")}});
  surface["schema"]="ssb.surface_runtime.v1";surface["recipe"]=Entry("recipe.json");surface.erase("tiles");Json("surface.json",surface);
  scene["surface"]=Entry("surface.json");SaveScene();
  StageBAssets assets(c);ASSERT_TRUE(assets.recipe);CudaSurfaceRecipe gpu(*assets.recipe);
  SurfaceTexel* device=nullptr;ASSERT_EQ(cudaMalloc(&device,assets.tile_bytes),cudaSuccess);
  std::vector<SurfaceTexel> actual(assets.side*assets.side),again(actual.size());
  for(unsigned id:{0u,2u,9u,11u}){
    gpu.Generate(id,device,nullptr);ASSERT_EQ(cudaMemcpy(actual.data(),device,assets.tile_bytes,cudaMemcpyDeviceToHost),cudaSuccess);
    auto expected=assets.ReadTile(id);
    for(size_t i=0;i<actual.size();++i){EXPECT_LE(std::abs(int(actual[i].albedo)-expected[i].albedo),1);
      EXPECT_LE(std::abs(int(actual[i].roughness)-expected[i].roughness),1);
      EXPECT_LE(std::abs(int(actual[i].nx)-expected[i].nx),1);EXPECT_LE(std::abs(int(actual[i].nq)-expected[i].nq),1);}
    gpu.Generate((id+1)%12,device,nullptr);gpu.Generate(id,device,nullptr);
    ASSERT_EQ(cudaMemcpy(again.data(),device,assets.tile_bytes,cudaMemcpyDeviceToHost),cudaSuccess);
    EXPECT_EQ(0,std::memcmp(actual.data(),again.data(),assets.tile_bytes));
  }
  cudaFree(device);
  auto first=assets.ReadTile(0),last=assets.ReadTile(9);
  // First bottom gutter exactly wraps to final core's last two rows.
  EXPECT_EQ(0,std::memcmp(first.data(),last.data()+16*20,2*20*8));
  std::vector<RowJob> jobs(8);for(size_t i=0;i<jobs.size();++i){jobs[i].pose.x=3+i*.6;jobs[i].pose.theta=i*.7;}
  OptixRenderer full(c,DefaultPtxPath(),8),split(c,DefaultPtxPath(),3);
  std::vector<uint8_t> a,b;std::vector<double> hits;full.Render(jobs,a,hits);
  for(size_t i=0;i<jobs.size();i+=3){std::vector<RowJob> part(jobs.begin()+i,jobs.begin()+std::min(jobs.size(),i+3));
    std::vector<uint8_t> pixels;split.Render(part,pixels,hits);b.insert(b.end(),pixels.begin(),pixels.end());}
  EXPECT_EQ(a,b);
  EXPECT_TRUE(full.Describe().at("runtime_surface_recipe"));
  // Payload identity must fail before use, even when file length is unchanged.
  source[0].albedo^=1;Binary("native.bin",source);EXPECT_THROW(StageBAssets invalid(c),std::runtime_error);
}

TEST_F(SurfaceFixture, AreaSamplingPreservesSubmillimetreWidthAndExposureIntegral) {
  scene["sampling"]["area_axis_samples"]=8;scene["sampling"]["time_samples"]=8;SaveScene();
  for(double width:{.0002,.0003,.0004,.0005,.0006}) {
  SetWidth(width);OptixRenderer renderer(c,DefaultPtxPath(),256);
  for(double omega:{0.,c.NominalOmega()}) {
    std::vector<RowJob> jobs(160);const double spacing=.000025;
    for(size_t i=0;i<jobs.size();++i) {jobs[i].pose.x=8;jobs[i].pose.theta=(.5+(double(i)-79.5)*spacing)/2.75;jobs[i].pose.omega=omega;}
    std::vector<uint8_t> pixels;std::vector<double> hits;renderer.Render(jobs,pixels,hits);
    double sum=0;for(auto code:pixels) sum+=(128.-code)/(128.-9.);
    double measured=sum/c.width*spacing;
    EXPECT_NEAR(measured,width,.000010) << "width="<<width<<", omega="<<omega;
    EXPECT_EQ(*std::max_element(pixels.begin(),pixels.end()),128);
  }
  }
}

TEST_F(SurfaceFixture, IntegratedPixelPreservesPhysicalWidthsWithMotion) {
  scene["sampling"]["area_axis_samples"]=8;scene["sampling"]["time_samples"]=3;
  scene["sampling"]["integrated_cracks"]=true;SaveScene();
  for(double width:{.0002,.0003,.0004,.0005,.0006}) {
    SetWidth(width);OptixRenderer renderer(c,DefaultPtxPath(),256);
    EXPECT_TRUE(renderer.Describe().at("integrated_cracks"));
    for(double omega:{0.,c.NominalOmega()}) {
      std::vector<RowJob> jobs(160);const double spacing=.000025;
      for(size_t i=0;i<jobs.size();++i) {
        jobs[i].pose.x=8;jobs[i].pose.theta=(.5+(double(i)-79.5)*spacing)/2.75;
        jobs[i].pose.omega=omega;jobs[i].pose.v=.3515625;
      }
      std::vector<uint8_t> pixels;std::vector<double> hits;renderer.Render(jobs,pixels,hits);
      double sum=0;for(auto code:pixels)sum+=(128.-code)/(128.-9.);
      EXPECT_NEAR(sum/c.width*spacing,width,.000005)<<"width="<<width<<", omega="<<omega;
    }
  }
}

TEST(CrackIntegral, HalfPlaneVolumeMatchesIndependentPolygonClipping) {
  // Clip the unit pixel square, then integrate its area over time using Simpson.
  // This oracle does not use the convolution/inclusion-exclusion formula.
  auto area=[](double a,double b,double bias) {
    using P=std::array<double,2>;
    std::vector<P> polygon{{-.5,-.5},{.5,-.5},{.5,.5},{-.5,.5}},clipped;
    for(unsigned i=0;i<polygon.size();++i) {
      P from=polygon[i],to=polygon[(i+1)%polygon.size()];
      double f=a*from[0]+b*from[1]+bias,g=a*to[0]+b*to[1]+bias;
      if(f<=0)clipped.push_back(from);
      if((f<=0)!=(g<=0)) {double t=f/(f-g);clipped.push_back({from[0]+t*(to[0]-from[0]),from[1]+t*(to[1]-from[1])});}
    }
    double sum=0;for(unsigned i=0;i<clipped.size();++i) {
      auto p=clipped[i],q=clipped[(i+1)%clipped.size()];sum+=p[0]*q[1]-p[1]*q[0];
    }
    return std::abs(sum)*.5;
  };
  for(auto weights:{std::array<double,3>{1,.3,.7},{-.2,.8,-.4},{1,0,.5},{1,1,0},{1e-8,1,.5}})
    for(double bias:{-.75,-.23,-.001,0.,.17,.6}) {
      const unsigned n=2048;double sum=0;
      for(unsigned i=0;i<=n;++i)sum+=(i==0||i==n?1:i%2?4:2)*area(weights[0],weights[1],weights[2]*(double(i)/n-.5)+bias);
      EXPECT_NEAR(BoxHalfPlane(weights[0],weights[1],weights[2],bias),sum/(3*n),5e-7);
    }
  EXPECT_DOUBLE_EQ(BoxHalfPlane(0,0,0,-1),1);
  EXPECT_DOUBLE_EQ(BoxHalfPlane(0,0,0,1),0);
}

TEST_F(SurfaceFixture, EvictionAndBatchSplittingPreserveEveryByte) {
  OptixRenderer renderer(c,DefaultPtxPath(),300);
  std::vector<RowJob> jobs(300);
  for(size_t i=0;i<jobs.size();++i) {jobs[i].pose.x=2.+16.*i/jobs.size();jobs[i].pose.theta=-3.1+6.2*i/jobs.size();jobs[i].record.sequence=i;}
  std::vector<uint8_t> whole,pieces,part;std::vector<double> all_hits,hit_pieces,hits;
  renderer.Render(jobs,whole,all_hits);
  for(size_t first=0;first<jobs.size();first+=37) {
    renderer.Render(std::vector<RowJob>(jobs.begin()+first,jobs.begin()+std::min(first+37,jobs.size())),part,hits);
    pieces.insert(pieces.end(),part.begin(),part.end());hit_pieces.insert(hit_pieces.end(),hits.begin(),hits.end());
  }
  EXPECT_EQ(whole,pieces);EXPECT_EQ(all_hits,hit_pieces);
  auto description=renderer.Describe();
  EXPECT_LE(description.at("texture_allocated_peak_bytes").get<size_t>(),12800u);
  EXPECT_GT(description.at("cpu_texture_allocated_peak_bytes").get<size_t>(),0u);
  EXPECT_LE(description.at("cpu_texture_allocated_peak_bytes").get<size_t>(),3200u);
  EXPECT_GT(description.at("tile_loads").get<size_t>(),4u);
}

TEST_F(SurfaceFixture, LampFootprintHasTheSpecifiedWidth) {
  scene["lamp"]["enabled"]=true;SaveScene();
  c.fov_at_nominal_m=1.8;  // inspection optic: extend beyond the nominal camera's 0.85 m FOV
  OptixRenderer renderer(c,DefaultPtxPath(),400);
  std::vector<uint8_t> pixels;std::vector<double> hits;
  RowJob centre{};centre.pose.x=8;centre.pose.theta=.2;renderer.Render({centre},pixels,hits);
  int peak=*std::max_element(pixels.begin(),pixels.end()),first=-1,last=-1;
  for(int u=0;u<c.width;++u) if(pixels[u]>=peak*.5) {if(first<0)first=u;last=u;}
  EXPECT_NEAR((last-first+1)*c.fov_at_nominal_m/c.width,1.2,.06);
  // Turn the inspection line 90 degrees to inspect the transverse beam while
  // keeping the lamp pose fixed. Rotating camera+lamp would always see its centre.
  c.truth.mount.twist_rad=pi/2;c.fov_at_nominal_m=.2;
  OptixRenderer transverse(c,DefaultPtxPath(),1);
  transverse.Render({centre},pixels,hits);peak=*std::max_element(pixels.begin(),pixels.end());
  first=last=-1;
  for(int u=0;u<c.width;++u) if(pixels[u]>=peak*.5) {if(first<0)first=u;last=u;}
  EXPECT_NEAR((last-first+1)*c.fov_at_nominal_m/c.width,.12,.006);
}

TEST_F(SurfaceFixture, ChangedTileIsRejectedInsteadOfBeingRendered) {
  std::fstream file(root/"tile_0.bin",std::ios::in|std::ios::out|std::ios::binary);file.put('x');file.close();
  OptixRenderer renderer(c,DefaultPtxPath(),1);RowJob job{};job.pose.x=0;job.pose.theta=-3.;
  std::vector<uint8_t> pixels;std::vector<double> hits;
  EXPECT_THROW(renderer.Render({job},pixels,hits),std::runtime_error);
}

TEST_F(SurfaceFixture, ParallelCobBeamFollowsAxialOffsetAndCoversCamera) {
  c.fov_at_nominal_m=1.8;
  scene["lamp"]["enabled"]=true;
  scene["lamp"]["samples"]=4;
  scene["lamp"]["length_m"]=.02;scene["lamp"]["width_m"]=.02;
  scene["lamp"]["offset_tangential_m"]=0.;scene["lamp"]["offset_radial_m"]=0.;
  RowJob row{};row.pose.x=8;row.pose.theta=.2;
  for(double offset:{-.115,.115}) {
    scene["lamp"]["offset_axial_m"]=offset;SaveScene();
    OptixRenderer renderer(c,DefaultPtxPath(),1);
    std::vector<uint8_t> pixels;std::vector<double> hits;renderer.Render({row},pixels,hits);
    int peak=*std::max_element(pixels.begin(),pixels.end()),first=-1,last=-1;
    for(int u=0;u<c.width;++u)if(pixels[u]>=peak*.5){if(first<0)first=u;last=u;}
    const auto x=[&](double u){return ((u+.5)/c.width-.5)*c.fov_at_nominal_m;};
    EXPECT_NEAR((x(first)+x(last))/2,offset,.015);
    EXPECT_LT(x(first),-.8522592711111112/2);
    EXPECT_GT(x(last),.8522592711111112/2);
    EXPECT_NEAR(x(last)-x(first),1.2,.06);
  }
  scene["lamp"]["samples"]=3;SaveScene();
  EXPECT_THROW(StageBAssets{c},std::runtime_error);
}
}

namespace {
TEST_F(SurfaceFixture, AdaptiveSamplingAndVisibilityMatchTheFullRayReference) {
  scene["sampling"]["area_axis_samples"]=4;scene["lamp"]["enabled"]=true;scene["lamp"]["shadows"]=true;SaveScene();
  std::vector<RowJob> jobs(120);
  for(size_t i=0;i<jobs.size();++i) {
    jobs[i].pose.x=8.+(double(i)-59.5)*.3515625/50000;
    jobs[i].pose.theta=(.5+(double(i)-59.5)*.000025)/2.75;jobs[i].pose.v=.3515625;jobs[i].pose.omega=c.NominalOmega();
  }
  std::vector<uint8_t> full,fast;std::vector<double> full_hits,fast_hits;
  {OptixRenderer renderer(c,DefaultPtxPath(),120);renderer.Render(jobs,full,full_hits);}
  scene["sampling"]["adaptive_area"]=true;
  scene["convex_panel_visibility"]=true;SaveScene();
  {OptixRenderer renderer(c,DefaultPtxPath(),120);EXPECT_TRUE(renderer.Describe().at("adaptive_area"));renderer.Render(jobs,fast,fast_hits);}
  EXPECT_EQ(full_hits,fast_hits);int worst=0;size_t substantial=0;
  for(size_t i=0;i<full.size();++i) {int error=std::abs(int(full[i])-fast[i]);worst=std::max(worst,error);substantial+=error>4;}
  EXPECT_LE(worst,8);EXPECT_LT(substantial,double(full.size())*.001);
}
}
