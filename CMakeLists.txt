cmake_minimum_required(VERSION 3.8.0)
project(vedisp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-non-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Ofast -fPIC -flto=auto")

# fmt
add_subdirectory(${CMAKE_SOURCE_DIR}/external/fmt EXCLUDE_FROM_ALL)

# CDT
add_subdirectory(${CMAKE_SOURCE_DIR}/external/CDT/CDT CDT)

# predicates
add_subdirectory(${CMAKE_SOURCE_DIR}/external/predicates)
include_directories(${CMAKE_SOURCE_DIR}/external/predicates/include)


include_directories(${CMAKE_SOURCE_DIR}/include)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_subdirectory(src)
