find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(KALMAN_FILTER SHARED
  ${ALL_SOURCE}
)

target_link_libraries(KALMAN_FILTER
  ${EIGEN3_LIBRARIES}
  ${Sophus_LIBRARIES}
)