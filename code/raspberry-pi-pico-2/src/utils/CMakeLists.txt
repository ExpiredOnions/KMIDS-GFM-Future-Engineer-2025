file(GLOB UTILS_SOURCES "*.cpp")
file(GLOB UTILS_HEADERS "*.h" "*.hpp")

add_library(utils STATIC ${UTILS_SOURCES} ${UTILS_HEADERS})

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
