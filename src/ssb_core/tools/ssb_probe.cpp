// Optical inspection/throughput probe, deliberately separate from encoder capture.
#include <chrono>
#include <fstream>
#include <iostream>
#include "ssb_core/optix_renderer.hpp"
#include "ssb_core/sha256.hpp"

int main(int argc,char** argv) {
  try {
    std::string config_path,output;int rows=1024;double x=8,theta=.2,omega=0,speed=0,rate=50000;
    for(int i=1;i<argc;++i) {
      std::string arg=argv[i];if(++i>=argc) throw std::runtime_error("missing option value");std::string value=argv[i];
      if(arg=="--config") config_path=value;else if(arg=="--output") output=value;
      else if(arg=="--rows") rows=std::stoi(value);else if(arg=="--x") x=std::stod(value);
      else if(arg=="--theta") theta=std::stod(value);else if(arg=="--omega") omega=std::stod(value);
      else if(arg=="--speed") speed=std::stod(value);else if(arg=="--rate") rate=std::stod(value);
      else throw std::runtime_error("unknown argument "+arg);
    }
    if(config_path.empty()||rows<1||rows>200000||rate<=0) throw std::runtime_error("invalid probe arguments");
    auto c=ssb::Config::Load(config_path);ssb::OptixRenderer renderer(c,ssb::DefaultPtxPath(),c.batch_rows);
    auto check=renderer.SelfCheck();std::ofstream pgm,hit_file;
    if(!output.empty()) {
      if(std::filesystem::exists(output)) throw std::runtime_error("probe output exists");
      std::filesystem::create_directories(output);
      pgm.open(std::filesystem::path(output)/"image.pgm",std::ios::binary);
      pgm<<"P5\n"<<c.width<<" "<<rows<<"\n255\n";
      hit_file.open(std::filesystem::path(output)/"hits.bin",std::ios::binary);
    }
    auto start=std::chrono::steady_clock::now();
    uint64_t checksum=0;int min_code=255,max_code=0;
    for(int first=0;first<rows;first+=c.batch_rows) {
      std::vector<ssb::RowJob> jobs(std::min(c.batch_rows,rows-first));
      for(size_t j=0;j<jobs.size();++j) {
        double dt=(first+j-.5*(rows-1))/rate;
        jobs[j].record.sequence=first+j;
        jobs[j].pose.x=x+speed*dt;jobs[j].pose.v=speed;
        jobs[j].pose.theta=theta+omega*dt;jobs[j].pose.omega=omega;
      }
      std::vector<uint8_t> pixels;std::vector<double> hits;renderer.Render(jobs,pixels,hits);
      for(auto v:pixels){checksum+=v;min_code=std::min(min_code,int(v));max_code=std::max(max_code,int(v));}
      if(pgm.is_open()) {pgm.write(reinterpret_cast<char*>(pixels.data()),pixels.size());
        hit_file.write(reinterpret_cast<char*>(hits.data()),hits.size()*sizeof(double));}
    }
    double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    size_t peak=0;std::ifstream status("/proc/self/status");std::string line;
    while(std::getline(status,line)) if(line.rfind("VmHWM:",0)==0) peak=std::stoull(line.substr(6))*1024;
    nlohmann::json result={{"schema","ssb.optical_probe.v1"},{"purpose","optical inspection; not encoder capture"},
      {"optical_signature",c.OpticalSignature()},
      {"rows",rows},{"wall_seconds",seconds},{"rows_per_second",rows/seconds},{"nominal_imaging_rtf",rows/rate/seconds},
      {"peak_rss_bytes",peak},{"checksum",checksum},{"min_code",min_code},{"max_code",max_code},
      {"x_mid_m",x},{"theta_mid_rad",theta},{"omega_rad_s",omega},{"speed_m_s",speed},{"row_rate_hz",rate},
      {"config_file",config_path},{"config_sha256",ssb::Sha256File(config_path)},
      {"describe",renderer.Describe()},{"self_check",check}};
    if(pgm.is_open()) {
      pgm.close();hit_file.close();if(pgm.fail()||hit_file.fail()) throw std::runtime_error("probe write failed");
      result["image_sha256"]=ssb::Sha256File(std::filesystem::path(output)/"image.pgm");
      result["hits_sha256"]=ssb::Sha256File(std::filesystem::path(output)/"hits.bin");
      std::ofstream(std::filesystem::path(output)/"probe.json")<<result.dump(2)<<"\n";
    }
    std::cout<<result.dump(2)<<std::endl;return 0;
  } catch(const std::exception& e) {std::cerr<<"ssb_probe: "<<e.what()<<std::endl;return 1;}
}
