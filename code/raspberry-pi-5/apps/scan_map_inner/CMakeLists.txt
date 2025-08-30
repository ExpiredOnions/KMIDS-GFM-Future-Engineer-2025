add_executable(scan_map_inner main.cpp)
target_include_directories(scan_map_inner PRIVATE)
target_link_libraries(
  scan_map_inner PRIVATE lidar_module lidar_processor pico2_module
                         combined_processor camera_module pid_controller)
