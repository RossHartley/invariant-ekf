# Specify the minimum version for CMake
cmake_minimum_required(VERSION 2.8.3)
# Project's name
project(inekf)

option(USE_CPP11 "Use flag -std=c++11" ON)
option(USE_MUTEX "Use mutex in code" OFF)

message("USE_CPP11: " ${USE_CPP11})


# Set compiler flags
if (USE_CPP11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(USE_CPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
SET(CMAKE_CXX_COMPILER /usr/bin/g++)

message("CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
unset(USE_CPP11)
unset(USE_MUTEX)

# Set the output folder where your program will be created  
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

############################################
#  The following folders will be included  #
############################################
include_directories("${PROJECT_SOURCE_DIR}/include/")

# Threading
find_package (Threads)

# Boost
find_package(Boost 1.57 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# Adding all classes
file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

######################
#  Add Execuatables  #
######################
link_directories(${PROJECT_SOURCE_DIR}/${Boost_LIBRARY_DIRS})

add_executable(landmarks ${PROJECT_SOURCE_DIR}/src/examples/landmarks.cpp ${src_files})
add_executable(kinematics ${PROJECT_SOURCE_DIR}/src/examples/kinematics.cpp ${src_files})
add_executable(propagation_speed ${PROJECT_SOURCE_DIR}/src/tests/propagation_speed.cpp ${src_files})
add_executable(correction_speed ${PROJECT_SOURCE_DIR}/src/tests/correction_speed.cpp ${src_files})
add_executable(large_state_speed ${PROJECT_SOURCE_DIR}/src/tests/large_state_speed.cpp ${src_files})
add_executable(left_vs_right_error_dynamics ${PROJECT_SOURCE_DIR}/src/tests/left_vs_right_error_dynamics.cpp ${src_files})
add_executable(NavState_tests ${PROJECT_SOURCE_DIR}/src/tests/NavState_tests.cpp ${src_files})

target_link_libraries(landmarks ${Boost_LIBRARIES})
target_link_libraries(kinematics ${Boost_LIBRARIES})
target_link_libraries(propagation_speed ${Boost_LIBRARIES})
target_link_libraries(correction_speed ${Boost_LIBRARIES})
target_link_libraries(large_state_speed ${Boost_LIBRARIES})
target_link_libraries(left_vs_right_error_dynamics ${Boost_LIBRARIES})
target_link_libraries(NavState_tests ${Boost_LIBRARIES})

# Create Library
add_library(inekf SHARED
    "${PROJECT_SOURCE_DIR}/src/InEKF.cpp"
    "${PROJECT_SOURCE_DIR}/src/RobotState.cpp"
    "${PROJECT_SOURCE_DIR}/src/LieGroup.cpp"
    "${PROJECT_SOURCE_DIR}/src/NoiseParams.cpp"
    "${PROJECT_SOURCE_DIR}/src/Observations.cpp"
    "${PROJECT_SOURCE_DIR}/src/NavState.cpp"
)
set_target_properties(inekf PROPERTIES LIBRARY_OUTPUT_NAME inekf)



#=============================================
# to allow find_package() on inekf
#=============================================
# 
# the following case be used in an external project requiring inekf:
#  ...
#  find_package(inekf) 
#  include_directories(${inekf_INCLUDE_DIRS}) 
#  ...

# NOTE: the following will support find_package for 1) local build (make) and 2) for installed files (make install)

# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE inekf)

# Create variable for the local build tree
#set_target_properties(inekf PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
get_property(inekf_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
get_property(inekf_library_dirs TARGET inekf PROPERTY LIBRARY_OUTPUT_DIRECTORY)
get_property(inekf_libraries TARGET inekf PROPERTY LIBRARY_OUTPUT_NAME)

message("inekf_include_dirs: " ${inekf_include_dirs})
message("inekf_library_dirs: " ${inekf_library_dirs})
message("inekf_libraries: " ${inekf_libraries})

# Configure config file for local build tree
configure_file(inekfConfig.cmake.in
  "${PROJECT_BINARY_DIR}/inekfConfig.cmake" @ONLY)

message("PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

# # 2- installation build #

# # Change the include location for the case of an install location
# set(inekf_include_dirs ${CMAKE_INSTALL_PREFIX}/include ${EIGEN_INCLUDE_DIR})

# # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
# configure_file(inekfConfig.cmake.in
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/inekfConfig.cmake" @ONLY)

# install(FILES
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/inekfConfig.cmake"
#   DESTINATION share/inekf/cmake COMPONENT dev)
