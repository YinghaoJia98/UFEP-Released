cmake_minimum_required(VERSION 3.0.2)
project(vrmapping)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  vrmapping_msgs
)
find_package(Eigen3 REQUIRED)

# GPU architectures
# By default this flag is NOT set. Cmake then detects the architecture of the build computer
# and compiles for that architecture only. This can be an issue if you're building on one
# machine, and running on machines with a different GPU achitecture. In this case, set the flag.
# The penalty for doing this is increased build times.
option(BUILD_FOR_ALL_ARCHS "Build for all GPU architectures" OFF)
# Only used if the BUILD_FOR_ALL_ARCHS flag above is true.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
    set(CUDA_ARCHITECTURE_FLAGS "86;80;75;72;70;61;60")
else()
    set(CUDA_ARCHITECTURE_FLAGS " -gencode=arch=compute_86,code=sm_86 \
                                  -gencode=arch=compute_80,code=sm_80 \
                                  -gencode=arch=compute_75,code=sm_75 \
                                  -gencode=arch=compute_72,code=sm_72 \
                                  -gencode=arch=compute_70,code=sm_70 \
                                  -gencode=arch=compute_61,code=sm_61 \
                                  -gencode=arch=compute_60,code=sm_60")
endif()
# Suppress spammy Eigen CUDA warnings.
# "expt-relaxed-constexpr" allows sharing constexpr between host and device code.
# "display_error_number" shows a warning number with all warnings, and the
# rest is just suppressing specific warnings from Eigen. Note that the numbers
# keep changing with every CUDA release so this list is a bit arbitrary.
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda --expt-relaxed-constexpr --disable-warnings --generate-line-info -lineinfo -Xcudafe --display_error_number")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")
# Find CUDA package
find_package(CUDA REQUIRED)



catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES vrmapping
  # CATKIN_DEPENDS geometry_msgs grid_map_core grid_map_msgs grid_map_ros pcl_ros roscpp rospy std_msgs tf tf2 tf2_ros
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########
include_directories(

  # include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}  # Include CUDA headers
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

# Add CUDA source files for the integrator
cuda_add_library(CellsIntegratorGPU
  src/Integrator/CellsIntegrator.cu  # Your CUDA file
)
set_target_properties(CellsIntegratorGPU PROPERTIES CUDA_SEPARABLE_COMPILATION ON)


# # Declare a C++ executable
file(GLOB src_files
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/Integrator/*.cpp"
)

# Use CUDA_NVCC_FLAGS for setting up CUDA compilation flags
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17 -O3 --expt-relaxed-constexpr")

# Add executable and link libraries
add_executable(${PROJECT_NAME}_node src/vrmapping_node.cpp ${src_files})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  CellsIntegratorGPU
  glog
  ${CUDA_LIBRARIES}
)

# add_executable(${PROJECT_NAME}_node src/vrmapping_node.cpp ${src_files})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   glog
# )
