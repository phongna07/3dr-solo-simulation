cmake_minimum_required(VERSION 3.16)
project(solo_camera_gimbal LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-gui8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

add_library(SoloWorldStabilizedCamera SHARED WorldStabilizedCamera.cc)
target_compile_features(SoloWorldStabilizedCamera PRIVATE cxx_std_17)
target_compile_options(SoloWorldStabilizedCamera PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(SoloWorldStabilizedCamera PRIVATE
  gz-sim8::gz-sim8 gz-rendering8::gz-rendering8 gz-plugin2::register)
install(TARGETS SoloWorldStabilizedCamera LIBRARY DESTINATION lib)

qt5_add_resources(coverage_resources coverage.qrc)
add_library(SoloCameraCoverageGui SHARED CameraCoverageGui.cc ${coverage_resources})
target_compile_features(SoloCameraCoverageGui PRIVATE cxx_std_17)
target_compile_options(SoloCameraCoverageGui PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(SoloCameraCoverageGui PRIVATE
  gz-gui8::gz-gui8 gz-rendering8::gz-rendering8 gz-plugin2::register Qt5::Quick)
install(TARGETS SoloCameraCoverageGui LIBRARY DESTINATION lib)
