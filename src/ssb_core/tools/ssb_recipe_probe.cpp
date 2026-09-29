// Compare generated material texels before lighting. Not a capture/MTF benchmark.
#include "ssb_core/surface_recipe.hpp"
#include "ssb_core/sha256.hpp"
#include <cuda_runtime.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>

int main(int argc,char** argv){try{
  std::string surface,output,list;for(int i=1;i<argc;++i){std::string k=argv[i];if(++i>=argc)throw std::runtime_error("missing argument");
    if(k=="--surface")surface=argv[i];else if(k=="--output")output=argv[i];else if(k=="--tiles")list=argv[i];else throw std::runtime_error("unknown argument");}
  if(surface.empty()||output.empty()||list.empty()||std::filesystem::exists(output))throw std::runtime_error("invalid/existing output");
  nlohmann::json s;std::ifstream(surface)>>s;ssb::SurfaceRecipe recipe(surface,s);ssb::CudaSurfaceRecipe gpu(recipe);
  std::filesystem::create_directories(output);unsigned side=recipe.grid.core+2*recipe.grid.gutter;size_t bytes=size_t(side)*side*8;
  ssb::SurfaceTexel* device=nullptr;if(cudaMalloc(&device,bytes)!=cudaSuccess)throw std::runtime_error("allocation failed");
  nlohmann::json report={{"purpose","material equivalence; not optical/encoder capture"},{"surface_sha256",ssb::Sha256File(surface)},
    {"source_device_bytes",gpu.Bytes()},{"tiles",nlohmann::json::array()}};
  std::istringstream parser(list);std::string word;bool passed=true;
  while(std::getline(parser,word,',')){unsigned id=std::stoul(word);
    if(id>=s.at("tiles_xq")[0].get<unsigned>()*s.at("tiles_xq")[1].get<unsigned>())throw std::runtime_error("tile out of range");
    auto start=std::chrono::steady_clock::now();gpu.Generate(id,device,nullptr);double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::vector<ssb::SurfaceTexel> actual(size_t(side)*side);if(cudaMemcpy(actual.data(),device,bytes,cudaMemcpyDeviceToHost)!=cudaSuccess)throw std::runtime_error("readback failed");
    auto expected=recipe.Generate(id);int maxdiff[4]={};size_t changed=0;
    for(size_t i=0;i<actual.size();++i){auto a=actual[i],b=expected[i];int diffs[]={std::abs(int(a.albedo)-b.albedo),std::abs(int(a.roughness)-b.roughness),std::abs(int(a.nx)-b.nx),std::abs(int(a.nq)-b.nq)};
      bool difference=false;for(int c=0;c<4;++c){maxdiff[c]=std::max(maxdiff[c],diffs[c]);difference|=diffs[c]!=0;}changed+=difference;
    }
    passed &= maxdiff[0]<=1&&maxdiff[1]<=1&&maxdiff[2]<=1&&maxdiff[3]<=1;
    auto file=std::filesystem::path(output)/("tile_"+std::to_string(id)+".bin");std::ofstream stream(file,std::ios::binary);stream.write(reinterpret_cast<char*>(actual.data()),bytes);stream.close();
    report["tiles"].push_back({{"index",id},{"generation_seconds",seconds},{"max_channel_delta",{maxdiff[0],maxdiff[1],maxdiff[2],maxdiff[3]}},{"different_texels",changed},{"sha256",ssb::Sha256File(file)}});
  }
  cudaFree(device);report["passed"]=passed;std::ofstream(std::filesystem::path(output)/"comparison.json")<<report.dump(2)<<"\n";
  std::cout<<report.dump(2)<<"\n";return passed?0:1;
}catch(const std::exception& e){std::cerr<<"ssb_recipe_probe: "<<e.what()<<"\n";return 1;}}
