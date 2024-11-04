cmake_minimum_required(VERSION 3.12)

# Projektname
project(VirtualImageGenerator)

# Setze den C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    # Set the floating-point model to 'precise' for better numerical stability
    add_compile_options(/fp:precise)

    # Optional: Disable some optimizations that could affect floating-point calculations
    add_compile_options(/Od)  # Disables optimizations for better debugging; remove in production
endif()

# OpenCV finden
set(OpenCV_DIR "C:/Bibliotheken/OpenCV/opencv_4_10_win/build") # Pfad zu deinem OpenCV-Build-Verzeichnis
find_package(OpenCV REQUIRED)

# Include-Verzeichnisse hinzufügen
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Quellcodedateien erfassen
file(GLOB SOURCES "src/*.cpp")  # Alle .cpp-Dateien im src-Verzeichnis

# Executable erstellen
add_executable(VirtualImageGenerator ${SOURCES})

# OpenCV-Bibliotheken verknüpfen
target_link_libraries(VirtualImageGenerator PRIVATE ${OpenCV_LIBS})

# Optionale: füge weitere spezifische Compiler-Flags oder Einstellungen hinzu
# set_target_properties(VirtualImageGenerator PROPERTIES COMPILE_FLAGS "-Wall")
