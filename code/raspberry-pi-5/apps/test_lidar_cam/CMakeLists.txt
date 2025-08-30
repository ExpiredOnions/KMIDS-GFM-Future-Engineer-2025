add_executable(test_lidar_cam main.cpp)
target_include_directories(test_lidar_cam PRIVATE)
target_link_libraries(test_lidar_cam PRIVATE lidar_module lidar_processor
                                             camera_module camera_processor)
