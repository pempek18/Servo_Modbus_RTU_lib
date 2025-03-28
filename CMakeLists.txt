cmake_minimum_required(VERSION 3.10)
project(Lichuan_RS_Libary)

set(CMAKE_CXX_STANDARD 17)

# Add the library
add_library(Lichuan_RS_Libary STATIC lichuan/LCDA630P_Modbus_RTU.cpp)

# Add the example program
add_executable(example_linux examples/main.cpp)

# Find Boost and link it
find_package(Boost REQUIRED COMPONENTS system serialization)
include_directories(${Boost_INCLUDE_DIRS})

# Link Boost to the library
target_link_libraries(Lichuan_RS_Libary PRIVATE ${Boost_LIBRARIES})

# Ensure example can use the library
target_include_directories(example_linux PRIVATE ${CMAKE_SOURCE_DIR}/lichuan)
target_link_libraries(example_linux PRIVATE Lichuan_RS_Libary)
