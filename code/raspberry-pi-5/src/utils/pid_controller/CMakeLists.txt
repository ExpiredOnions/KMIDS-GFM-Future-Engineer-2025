# NOTE: pid_controller

add_library(pid_controller STATIC pid_controller.cpp pid_controller.h)
target_include_directories(pid_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
