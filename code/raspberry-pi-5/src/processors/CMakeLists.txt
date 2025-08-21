# NOTE: lidar_processor

add_library(lidar_processor STATIC lidar_processor.cpp lidar_processor.h)
target_include_directories(
  lidar_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
                         ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(
  lidar_processor
  PRIVATE ${OpenCV_LIBS}
  PUBLIC utils)

# NOTE: camera_processor

add_library(camera_processor STATIC camera_processor.cpp camera_processor.h)
target_include_directories(
  camera_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
                          ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(
  camera_processor
  PRIVATE ${OpenCV_LIBS}
  PUBLIC utils)
