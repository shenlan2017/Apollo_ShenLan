file(GLOB ALL_SOURCE "*.cpp")

find_package(glog REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${glog_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GeographicLib_INCLUDE_DIRS})


add_library(SENSOR_DATA SHARED
  ${ALL_SOURCE}
)

target_link_libraries(SENSOR_DATA
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  glog
)