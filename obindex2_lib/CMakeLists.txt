cmake_minimum_required(VERSION 3.5)
project(obindex2_lib LANGUAGES CXX)

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

###########
## Build ##
###########

### ----- OBIndex2 Library

set(${PROJECT_NAME}_sources
	src/binary_descriptor.cc
        src/binary_tree_node.cc
        include/obindex2/priority_queues.h
        src/binary_tree.cc
        src/binary_index.cc)

include_directories(include)

# Compiling the library
add_library(${PROJECT_NAME}
	    ${${PROJECT_NAME}_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp OpenCV)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(OpenCV)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
