cmake_minimum_required(VERSION 3.12)

# project name
project(GIRAFFE)

# c++ standard and flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

if(MSVC)
    # Set the floating-point model to 'precise' for better numerical stability
    add_compile_options(/fp:precise)
endif()

# find opencv
set(OpenCV_DIR "C:/Bibliotheken/OpenCV/opencv_4_10_win/build") # set path to OPENCV
find_package(OpenCV REQUIRED)

# add include files
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add source files
file(GLOB SOURCES "src/*.cpp") 

# Executable erstellen
add_executable(GIRAFFE ${SOURCES})

# link opencv
target_link_libraries(GIRAFFE PRIVATE ${OpenCV_LIBS})

# Optional compiler flags
# set_target_properties(GIRAFFE PROPERTIES COMPILE_FLAGS "-Wall")
