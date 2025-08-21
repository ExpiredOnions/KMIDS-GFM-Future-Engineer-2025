add_executable(test_i2c main.cpp)
target_include_directories(test_i2c PRIVATE)
target_link_libraries(test_i2c PUBLIC i2c_master)
