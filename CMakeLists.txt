cmake_minimum_required(VERSION 3.10)
project(Lichuan_RS_Libary)

set(CMAKE_CXX_STANDARD 17)

# Detect operating system
if(WIN32)
    set(IS_WINDOWS TRUE)
    message(STATUS "Building for Windows")
elseif(UNIX AND NOT APPLE)
    set(IS_LINUX TRUE)
    message(STATUS "Building for Linux")
elseif(APPLE)
    set(IS_MACOS TRUE)
    message(STATUS "Building for macOS")
endif()

# Detect MSYS2/MinGW environment
if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(IS_MSYS2 TRUE)
    message(STATUS "Using MSYS2/MinGW compiler")
endif()

# Add the library
add_library(Lichuan_RS_Libary STATIC lichuan/LichuanMotion.cpp lichuan/LCDA630P.cpp lichuan/LCDA6.cpp)

# Windows-specific examples
if(IS_WINDOWS)
    add_executable(main_windows_LCDA6 examples/main_windows_LCDA6.cpp)
    message(STATUS "Added Windows native example")
endif()

# Find Boost and link it
if(IS_LINUX)
    message(STATUS "Added Linux examples")
    add_executable(example_linux examples/main_linux_LCDA630P.cpp)
    add_executable(example_linux_LCDA6 examples/main_linux_LCDA6.cpp)
    find_package(Boost REQUIRED COMPONENTS system serialization)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

# Link Boost to the library
target_link_libraries(Lichuan_RS_Libary PRIVATE ${Boost_LIBRARIES})

# Windows-specific libraries
if(IS_WINDOWS)
    # Add Windows-specific libraries for native Windows example
    if(IS_MSYS2)
        # For MSYS2, we might need to link against Windows libraries
        set(WINDOWS_LIBS kernel32 user32)
    else()
        # For MSVC
        set(WINDOWS_LIBS kernel32 user32)
    endif()
    message(STATUS "Windows libraries: ${WINDOWS_LIBS}")
endif()

# Ensure example can use the library
if(IS_LINUX)
    target_include_directories(example_linux PRIVATE ${CMAKE_SOURCE_DIR}/lichuan)
    target_link_libraries(example_linux PRIVATE Lichuan_RS_Libary)
    target_include_directories(example_linux_LCDA6 PRIVATE ${CMAKE_SOURCE_DIR}/lichuan)
    target_link_libraries(example_linux_LCDA6 PRIVATE Lichuan_RS_Libary)
endif()
# Windows-specific example configuration
if(IS_WINDOWS)
    target_include_directories(main_windows_LCDA6 PRIVATE ${CMAKE_SOURCE_DIR}/lichuan)
    target_link_libraries(main_windows_LCDA6 PRIVATE Lichuan_RS_Libary ${WINDOWS_LIBS})
endif()