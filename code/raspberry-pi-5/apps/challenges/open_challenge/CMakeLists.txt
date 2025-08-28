add_executable(open_challenge main.cpp)
target_include_directories(open_challenge PRIVATE)
target_link_libraries(open_challenge PRIVATE lidar_module lidar_processor
                                             pico2_module combined_processor)
