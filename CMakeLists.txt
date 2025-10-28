cmake_minimum_required(VERSION 3.16)

project(robotics-sim-controller VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robotics_core
    src/robot.cpp
    src/sensors.cpp
    src/telemetry.cpp
)

target_include_directories(robotics_core PUBLIC
    ${PROJECT_SOURCE_DIR}/src
)

if(MSVC)
    target_compile_options(robotics_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(robotics_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(robotics-sim src/main.cpp)
target_link_libraries(robotics-sim PRIVATE robotics_core)

if(MSVC)
    target_compile_options(robotics-sim PRIVATE /W4 /permissive-)
else()
    target_compile_options(robotics-sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(robotics-tests
        tests/pid_tests.cpp
        tests/kinematics_tests.cpp
    )
    target_link_libraries(robotics-tests PRIVATE robotics_core gtest_main)

    if(MSVC)
        target_compile_options(robotics-tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(robotics-tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(robotics-tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()