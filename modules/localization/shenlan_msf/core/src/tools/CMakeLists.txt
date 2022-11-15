find_package(glog REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${glog_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(TOOLS SHARED
  ${ALL_SOURCE}
)

target_link_libraries(TOOLS
  ${PCL_LIBRARIES}
  glog
)