# Specify the minimum version for CMake
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") # Debugging
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wl,--no-as-needed") 

# Project's name
project(inekf)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

############################################
#  The following folders will be included  #
############################################
include_directories("${PROJECT_SOURCE_DIR}/include/")

# Threading
find_package (Threads)

# Boost
find_package(Boost 1.58 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Adding all classes
file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

######################
#  Add Execuatables  #
######################
link_directories(${PROJECT_SOURCE_DIR}/${Boost_LIBRARY_DIRS})

add_executable(landmarks ${PROJECT_SOURCE_DIR}/src/examples/landmarks.cpp ${src_files})
add_executable(propagation_speed ${PROJECT_SOURCE_DIR}/src/tests/propagation_speed.cpp ${src_files})

target_link_libraries(landmarks ${Boost_LIBRARIES})
target_link_libraries(propagation_speed ${Boost_LIBRARIES})




