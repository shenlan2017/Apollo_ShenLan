add_subdirectory(cloud_filter)
add_subdirectory(pre_integrator)
add_subdirectory(registration)
add_subdirectory(scan_adjust)

if(BUILD_MAPPING)
  add_subdirectory(graph_optimizer)
else()
  if(USE_KALMAN)
    add_subdirectory(kalman_filter)
  else()
    add_subdirectory(sliding_window)
  endif()
endif()


