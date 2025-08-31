# NOTE: camera_module

add_library(camera_module STATIC camera_module.cpp camera_module.h)
target_include_directories(
  camera_module
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
         ${LIBCAMERA_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/external/lccv/include
         ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(
  camera_module
  PRIVATE ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES} liblccv
  PUBLIC ring_buffer logger)
