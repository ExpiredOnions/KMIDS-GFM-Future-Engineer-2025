# ~~~
# NOTE: Uncomment this when there is .cpp file
# file(GLOB UTILS_SOURCES "*.cpp")
# file(GLOB UTILS_HEADERS "*.h" "*.hpp")
# ~~~

add_library(utils INTERFACE)

# ~~~
# NOTE: Uncomment this when there is .cpp file
# add_library(utils STATIC ${UTILS_SOURCES} ${UTILS_HEADERS})
# ~~~

target_include_directories(utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
