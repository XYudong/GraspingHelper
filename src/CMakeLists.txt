cmake_minimum_required(VERSION 3.5.1)
project(3DSegmentation)

set(CMAKE_CXX_STANDARD 11)

# for preprocessor
add_definitions(${PCL_DEFINITIONS})

# include path for compiler
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( PCL 1.7 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )

#find_package(realsense2 REQUIRED )
#include_directories( ${realsense_INCLUDE_DIR} )

find_package(glfw3 3.2 REQUIRED)

set(sources main.cpp
#        cylinder_segmentation.cpp
        pclPlay.cpp
#        rsPlay.cpp
#        openGLHelper.cpp
        utils.cpp
        plane_segmentation.cpp)
add_executable(3DSegmentation ${sources})

#link_directories(${PCL_LIBRARY_DIRS})  // rarely necessary

# for linker
target_link_libraries(3DSegmentation
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
#       realsense2
        glfw
        )

