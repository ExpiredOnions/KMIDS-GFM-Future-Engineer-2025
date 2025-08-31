# NOTE: camera_processor

add_library(camera_processor STATIC camera_processor.cpp camera_processor.h)
target_include_directories(
  camera_processor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
                          ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(camera_processor PRIVATE ${OpenCV_LIBS})
