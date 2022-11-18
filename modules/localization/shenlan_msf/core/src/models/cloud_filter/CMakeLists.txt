find_package(glog REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${glog_INCLUDE_DIRS})
include_directories(${yaml-cpp_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(CLOUD_FILTER SHARED
  ${ALL_SOURCE}
)

target_link_libraries(CLOUD_FILTER
  ${PCL_LIBRARIES}
  glog
  yaml-cpp
)