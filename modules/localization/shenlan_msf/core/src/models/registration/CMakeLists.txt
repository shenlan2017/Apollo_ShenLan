find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

file(GLOB ALL_SOURCE "*.cpp")

add_library(REGISTRATION SHARED
  ${ALL_SOURCE}
)

target_link_libraries(REGISTRATION
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)