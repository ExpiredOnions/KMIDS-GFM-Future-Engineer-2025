# NOTE: lidar_processor

add_library(lidar_processor STATIC lidar_processor.cpp lidar_processor.h)
target_include_directories(
  lidar_processor
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
         ${CMAKE_SOURCE_DIR}/src/types ${CMAKE_SOURCE_DIR}/src/utils)
target_link_libraries(lidar_processor PRIVATE ${OpenCV_LIBS} utils)
