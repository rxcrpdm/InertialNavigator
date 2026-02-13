cmake_minimum_required(VERSION 3.16)
project(InertialNavigatorProject LANGUAGES CXX)

# ----------------------------------------
# C++ standard
# ----------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------
# Find Qt (Qt6 preferred, fallback to Qt5)
# ----------------------------------------
find_package(Qt6 COMPONENTS Core REQUIRED QUIET)

if (NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core REQUIRED)
    set(QT_LIBRARIES Qt5::Core)
else()
    set(QT_LIBRARIES Qt6::Core)
endif()

# ----------------------------------------
# Find Eigen
# ----------------------------------------
# Option 1: Use system-installed Eigen3
find_package(Eigen3 QUIET)

if (NOT Eigen3_FOUND)
    # Option 2: Use local Eigen in third_party/
    message(STATUS "Eigen3 not found system-wide. Using bundled version.")
    include_directories(${CMAKE_SOURCE_DIR}/third_party/eigen)
else()
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# ----------------------------------------
# Include directories
# ----------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# ----------------------------------------
# Source files
# ----------------------------------------
set(SOURCES
    src/InertialNavigator.cpp
    src/WheelOdometry.cpp
    src/EskfFusion.cpp
    src/main.cpp
)

# ----------------------------------------
# Create executable
# ----------------------------------------
add_executable(inertial_nav ${SOURCES})

target_link_libraries(inertial_nav PRIVATE ${QT_LIBRARIES})

# ----------------------------------------
# For Qt MOC (if signals/slots used)
# ----------------------------------------
set_target_properties(inertial_nav PROPERTIES
    AUTOMOC ON
    AUTOUIC OFF
    AUTORCC OFF
)

