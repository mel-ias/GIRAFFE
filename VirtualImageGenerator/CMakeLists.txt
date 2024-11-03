cmake_minimum_required(VERSION 3.12)

# Projektname
project(my_project)

# Setze den C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV finden
set(OpenCV_DIR "C:/Bibliotheken/OpenCV/opencv_4_10_win/build") # Pfad zu deinem OpenCV-Build-Verzeichnis
find_package(OpenCV REQUIRED)

# Include-Verzeichnisse hinzufügen
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Quellcodedateien erfassen
file(GLOB SOURCES "src/*.cpp")  # Alle .cpp-Dateien im src-Verzeichnis

# Executable erstellen
add_executable(my_project ${SOURCES})

# OpenCV-Bibliotheken verknüpfen
target_link_libraries(my_project PRIVATE ${OpenCV_LIBS})

# Optionale: füge weitere spezifische Compiler-Flags oder Einstellungen hinzu
# set_target_properties(my_project PROPERTIES COMPILE_FLAGS "-Wall")
