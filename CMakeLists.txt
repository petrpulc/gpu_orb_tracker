cmake_minimum_required(VERSION 3.8)
project(gpu_orb_tracker)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
add_executable(gpu_orb_tracker main.cpp Motion.cpp Motion.h settings.h Layer.h HomographyLayer.cpp HomographyLayer.h PointLayer.cpp PointLayer.h PointCloud.cpp PointCloud.h nanoflann.hpp Feature.cpp Feature.h VectorCloud.cpp VectorCloud.h)
target_link_libraries(gpu_orb_tracker ${OpenCV_LIBS})
