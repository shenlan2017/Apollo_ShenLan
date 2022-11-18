find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${glog_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(SCAN_ADJUST SHARED
  ${ALL_SOURCE}
)

target_link_libraries(SCAN_ADJUST
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  glog
)