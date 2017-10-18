project(obindex2_ros LANGUAGES CXX)
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp)

# Defining the package
catkin_package(
    INCLUDE_DIRS lib/include
    LIBRARIES obindex2
    CATKIN_DEPENDS roscpp
    DEPENDS
)

###########
## Build ##
###########

# Compiling the library
add_subdirectory(lib)