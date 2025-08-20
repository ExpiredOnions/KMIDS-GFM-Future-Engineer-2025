add_executable(log_viewer main.cpp)
target_include_directories(log_viewer
                           PRIVATE ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(log_viewer PRIVATE utils lidar_processor camera_processor)
