cmake_minimum_required(VERSION 2.8.11)

project(2d_map_to_3d)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library

find_package(VTK 6.1 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.3 REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})
