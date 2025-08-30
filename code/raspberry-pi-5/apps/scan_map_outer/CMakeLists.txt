add_executable(scan_map_outer main.cpp)
target_include_directories(scan_map_outer PRIVATE)
target_link_libraries(
  scan_map_outer PRIVATE lidar_module lidar_processor pico2_module
                         combined_processor pid_controller)
