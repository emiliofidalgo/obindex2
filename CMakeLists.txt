project(obindex2 LANGUAGES CXX)
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Compile tests or not
option(EnableTesting "EnableTesting" ON)

# Setting build type to Release
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Packages required to work with the library
find_package(OpenCV REQUIRED) # OpenCV
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED) # OpenMP
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include
                    external
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARIES}
                 ${Boost_LIBRARIES})

# Copy test image to the binary directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/image.jpg
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

### Targets ###
add_library(obindex2
            src/binary_descriptor.cc
            src/binary_tree_node.cc
            include/obindex2/priority_queues.h
            src/binary_tree.cc)
target_link_libraries(obindex2
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES})

### Testing ###
if (${EnableTesting} STREQUAL "ON")
  # Test for BinaryDescriptor class
  add_executable(test_bdesc tests/test_bdesc.cc)
  target_link_libraries(test_bdesc obindex2)

  # Test for BinaryTree class
  add_executable(test_btree tests/test_btree.cc)
  target_link_libraries(test_btree obindex2)

  enable_testing(true)  # Enables unit-testing.
  add_test(NAME test_bdesc COMMAND test_bdesc)
  add_test(NAME test_btree COMMAND test_btree)
endif()
