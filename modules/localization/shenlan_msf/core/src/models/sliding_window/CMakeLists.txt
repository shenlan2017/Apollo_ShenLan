find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(glog REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${glog_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(SLIDING_WINDOW SHARED
  ${ALL_SOURCE}
)

target_link_libraries(SLIDING_WINDOW
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Sophus_LIBRARIES}
  glog
)