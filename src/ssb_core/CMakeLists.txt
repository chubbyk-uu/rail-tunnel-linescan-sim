cmake_minimum_required(VERSION 3.18)
project(ssb_core LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# Timing, geometry and storage: no GPU dependency, so it is testable anywhere.
add_library(ssb_timing STATIC
  src/crossing.cpp src/config.cpp src/sha256.cpp src/timing.cpp src/kinematic.cpp src/camera_model.cpp src/stage_b_assets.cpp src/local_geometry.cpp src/surface_recipe.cpp)
set_target_properties(ssb_timing PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ssb_timing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_options(ssb_timing PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ssb_timing PUBLIC yaml-cpp nlohmann_json::nlohmann_json OpenSSL::Crypto)

# Source-tree identity baked in at every build (DESIGN.md §12.1).
set(SSB_STAMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/stamp")
add_custom_target(ssb_build_stamp ALL
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DBUILD_TYPE=${CMAKE_BUILD_TYPE}
          -DOUTPUT=${SSB_STAMP_DIR}/ssb_core/build_stamp.hpp -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_stamp.cmake
  BYPRODUCTS ${SSB_STAMP_DIR}/ssb_core/build_stamp.hpp
  COMMENT "Recording source tree state")

# The imaging backend is mandatory: a missing CUDA or OptiX is a configure error,
# never a silently reduced build (DESIGN.md §12.1).
set(SSB_OPTIX_SDK "$ENV{HOME}/opt/optix-sdk-9.1.0" CACHE PATH "OptiX SDK root")
if(NOT EXISTS "${SSB_OPTIX_SDK}/include/optix.h")
  message(FATAL_ERROR "OptiX SDK not found at SSB_OPTIX_SDK=${SSB_OPTIX_SDK}")
endif()
if(NOT CUDAToolkit_ROOT AND EXISTS "/usr/local/cuda")
  set(CUDAToolkit_ROOT "/usr/local/cuda")
endif()
find_package(CUDAToolkit REQUIRED)
set(CMAKE_CUDA_ARCHITECTURES 75)
set(CMAKE_CUDA_COMPILER "${CUDAToolkit_NVCC_EXECUTABLE}")
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(SSB_PTX "${CMAKE_CURRENT_BINARY_DIR}/ssb_scan.ptx")
add_custom_command(OUTPUT "${SSB_PTX}"
  COMMAND ${CUDAToolkit_NVCC_EXECUTABLE} --ptx -O3 -std=c++17 -arch=compute_75
          -I${SSB_OPTIX_SDK}/include -I${CMAKE_CURRENT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}/src/optix/scan.cu -o "${SSB_PTX}"
  DEPENDS src/optix/scan.cu src/optix/launch_params.h include/ssb_core/texture.hpp include/ssb_core/surface_types.hpp include/ssb_core/crack_integral.hpp
  COMMENT "Compiling OptiX PTX")
add_custom_target(ssb_ptx ALL DEPENDS "${SSB_PTX}")

add_library(ssb_core SHARED src/optix/optix_renderer.cpp src/optix/surface_recipe.cu src/session.cpp src/pipeline.cpp)
add_dependencies(ssb_core ssb_ptx ssb_build_stamp)
target_include_directories(ssb_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${SSB_OPTIX_SDK}/include ${SSB_STAMP_DIR})
target_compile_options(ssb_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic> $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
target_compile_definitions(ssb_core PRIVATE
  SSB_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  SSB_PTX_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ptx/ssb_scan.ptx")
target_link_libraries(ssb_core PUBLIC ssb_timing CUDA::cudart ${CMAKE_DL_LIBS} pthread)

add_executable(ssb_render tools/ssb_render.cpp)
add_executable(ssb_optical_identity tools/ssb_optical_identity.cpp)
target_link_libraries(ssb_optical_identity ssb_timing)
install(TARGETS ssb_optical_identity DESTINATION lib/${PROJECT_NAME})
target_link_libraries(ssb_render ssb_core)
add_executable(ssb_selfcheck tools/ssb_selfcheck.cpp)
target_link_libraries(ssb_selfcheck ssb_core)
add_executable(ssb_probe tools/ssb_probe.cpp)
target_link_libraries(ssb_probe ssb_core)
add_executable(ssb_recipe_probe tools/ssb_recipe_probe.cpp)
target_link_libraries(ssb_recipe_probe ssb_core)
install(TARGETS ssb_render ssb_selfcheck ssb_probe ssb_recipe_probe DESTINATION lib/${PROJECT_NAME})
install(TARGETS ssb_core EXPORT export_ssb_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(FILES "${SSB_PTX}" DESTINATION share/${PROJECT_NAME}/ptx)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ssb_timing EXPORT export_ssb_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_crossing test/test_crossing.cpp)
  target_link_libraries(test_crossing ssb_timing)
  ament_add_gtest(test_timing test/test_timing.cpp)
  target_link_libraries(test_timing ssb_timing)
  target_compile_definitions(test_timing PRIVATE SSB_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  ament_add_gtest(test_pipeline_failure test/test_pipeline_failure.cpp)
  target_link_libraries(test_pipeline_failure ssb_core)
  target_compile_definitions(test_pipeline_failure PRIVATE SSB_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # GPU tests run inside the OptiX runtime wrapper against the build-tree PTX.
  set(SSB_WRAPPER "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/with_optix_runtime.sh")
  ament_add_gtest_executable(test_render test/test_render.cpp test/test_surface_render.cpp)
  target_link_libraries(test_render ssb_core)
  target_compile_definitions(test_render PRIVATE SSB_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  set(SSB_RENDER_XML "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_render.gtest.xml")
  ament_add_test(test_render
    COMMAND bash ${SSB_WRAPPER} $<TARGET_FILE:test_render> --gtest_output=xml:${SSB_RENDER_XML}
    RESULT_FILE ${SSB_RENDER_XML}
    ENV SSB_PTX=${SSB_PTX} TIMEOUT 600)
endif()

ament_export_targets(export_ssb_core)
ament_export_dependencies(yaml-cpp nlohmann_json OpenSSL CUDAToolkit)
ament_package()
