// No GPU initialization: refuse an incompatible calibration before moving/capturing.
#include <fstream>
#include <iostream>
#include "ssb_core/config.hpp"
#include "ssb_core/timing.hpp"

int main(int argc, char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--record-layout") {
      std::cout<<nlohmann::json{{"pose_record_bytes",sizeof(ssb::PoseSample)},
                              {"row_job_bytes",sizeof(ssb::RowJob)}}.dump()<<'\n';
      return 0;
    }
    std::string config, calibration;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (i+1 == argc) throw std::runtime_error("missing option value");
      if (arg == "--config") config = argv[++i];
      else if (arg == "--calibration") calibration = argv[++i];
      else throw std::runtime_error("unknown option: " + arg);
    }
    if (config.empty()) throw std::runtime_error("--config required");
    const auto c = ssb::Config::Load(config);
    const auto signature = c.OpticalSignature();
    if (!calibration.empty()) {
      std::ifstream in(calibration);
      if (!in) throw std::runtime_error("cannot open calibration: " + calibration);
      nlohmann::json measured; in >> measured;
      if (measured.at("optical_signature") != signature)
        throw std::runtime_error("calibration optical signature mismatch; calibrate this rig before capture");
    }
    std::cout << signature << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
