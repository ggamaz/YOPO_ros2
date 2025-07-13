cmake_minimum_required(VERSION 3.10)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(CUDA REQUIRED)
enable_language(CUDA)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Query the GPU architecture
set(CUDA_ARCHITECTURES "86")

cuda_add_library(raycast_cuda SHARED src/sensor_simulator.cu)
target_compile_options(raycast_cuda PRIVATE -Xcompiler -fPIC)
target_include_directories(raycast_cuda PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(raycast_cuda 
  ${CUDA_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  # src/sensor_simulator.cpp
  src/test_simulator_cuda.cpp
  src/perlinnoise.cpp
  src/maps.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  OpenMP::OpenMP_CXX
  yaml-cpp
  raycast_cuda
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN SensorSimulator
  EXECUTABLE sensor_simulator_cuda
)

ament_auto_add_executable(dataset_generator
  src/dataset_generator.cpp
)
target_link_libraries(dataset_generator
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  OpenMP::OpenMP_CXX
  yaml-cpp
  raycast_cuda
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  pointcloud
)
