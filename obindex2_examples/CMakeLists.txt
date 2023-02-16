cmake_minimum_required(VERSION 3.5)
project(obindex2_examples LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Catkin dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(obindex2_lib REQUIRED)

###########
## Build ##
###########

### ----- OBIndex2 bdesc

add_executable(test_bdesc src/test_bdesc.cc
			  src/catch.hpp)
ament_target_dependencies(test_bdesc rclcpp OpenCV obindex2_lib)

### ----- OBIndex2 btree
add_executable(test_btree src/test_btree.cc
			  src/catch.hpp)
ament_target_dependencies(test_btree rclcpp OpenCV obindex2_lib)

### ----- OBIndex2 bindex
add_executable(test_bindex src/test_bindex.cc
			  src/catch.hpp)
ament_target_dependencies(test_bindex rclcpp OpenCV obindex2_lib)

install(TARGETS
  test_bdesc
  test_btree
  test_bindex
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
