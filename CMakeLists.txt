cmake_minimum_required(VERSION 3.10)
project(Lichuan_RS_Libary)

add_executable(Lichuan_RS_Libary main.cpp lichuan/LCDA630P_Modbus_RTU.cpp) # Add the source file here

find_package(Boost REQUIRED COMPONENTS system serialization)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(Lichuan_RS_Libary PRIVATE ${Boost_LIBRARIES})