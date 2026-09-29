// Pre-flight backend check (DESIGN.md §12.1): loads the installed PTX through the
// same renderer a capture uses and compares a probe row with an analytic answer.
#include <iostream>

#include "ssb_core/optix_renderer.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: ssb_selfcheck CONFIG [PTX]\n";
    return 2;
  }
  try {
    const auto config = ssb::Config::Load(argv[1]);
    ssb::OptixRenderer renderer(config, argc > 2 ? argv[2] : ssb::DefaultPtxPath(), 1);
    std::cout << nlohmann::json({{"describe", renderer.Describe()}, {"self_check", renderer.SelfCheck()}}).dump(2)
              << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ssb_selfcheck FAILED: " << e.what() << std::endl;
    return 1;
  }
}
