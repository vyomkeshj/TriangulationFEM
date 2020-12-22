cmake_minimum_required(VERSION 3.13)
project(RealCollaborationCal)

set(CMAKE_CXX_STANDARD 11)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
set (CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-lncurses")

# added -fopenmp
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")

find_package(VTK 8.2 REQUIRED)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
find_package(Qt5Core REQUIRED)

link_directories(${QT_LIBRARY_DIRS})

include(${VTK_USE_FILE})

set(SOURCES Main.cpp)

include_directories(include ./Header, ./Source)
add_executable(RealCollaborationCal Main.cpp Source/DelaunayTriangulation.cpp Source/DotCloudReader.cpp
        Source/DotCloudGenerator.cpp
        Source/Triangle.cpp Source/Vector3D.cpp Source/Visualization.cpp)

target_link_libraries(RealCollaborationCal ${VTK_LIBRARIES})

include_directories("/usr/local/include" "/usr/local/opt/llvm/include")
link_directories("/usr/local/lib" "/usr/local/opt/llvm/lib")


find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(RealCollaborationCal OpenMP::OpenMP_CXX)
endif()
