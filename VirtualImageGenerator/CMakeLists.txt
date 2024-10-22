cmake_minimum_required(VERSION 3.12)

project(VirtualImageGenerator)

# OpenCV finden
set(OpenCV_DIR "C:/Bibliotheken/OpenCV/opencv_4_10_win/build") # Pfad zu Ihrem OpenCV-Build-Verzeichnis
set(BOOST_DIR "C:/Bibliotheken/boost_1_73_0/boost_1_73_0") # Pfad zum Boost verzeichnis

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

# Projektinformationen und Ziel hinzufügen
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories( ${OpenCV_LIB_DIR} )

file(GLOB SOURCES "src/*.cpp")

target_link_libraries(VirtualImageGenerator PRIVATE ${OpenCV_LIBS})

# Options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)