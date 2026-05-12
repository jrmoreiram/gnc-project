# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Find Google Test
find_package(GTest QUIET)

# If Google Test is not found, download it
if(NOT GTEST_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# Define source files (excluding main.cpp)
set(GNC_SOURCES
  ${CMAKE_SOURCE_DIR}/src/gnc_system.cpp
  ${CMAKE_SOURCE_DIR}/src/sensors/imu.cpp
  ${CMAKE_SOURCE_DIR}/src/sensors/gps.cpp
  ${CMAKE_SOURCE_DIR}/src/sensors/barometer.cpp
  ${CMAKE_SOURCE_DIR}/src/estimation/ekf.cpp
  ${CMAKE_SOURCE_DIR}/src/estimation/state_estimator.cpp
  ${CMAKE_SOURCE_DIR}/src/guidance/guidance_law.cpp
  ${CMAKE_SOURCE_DIR}/src/guidance/trajectory_planner.cpp
  ${CMAKE_SOURCE_DIR}/src/control/pid_controller.cpp
  ${CMAKE_SOURCE_DIR}/src/control/actuator_controller.cpp
  ${CMAKE_SOURCE_DIR}/src/control/rate_controller.cpp
  ${CMAKE_SOURCE_DIR}/src/flight_loop/flight_loop.cpp
  ${CMAKE_SOURCE_DIR}/src/utils/math_utils.cpp
  ${CMAKE_SOURCE_DIR}/src/utils/data_logger.cpp
  ${CMAKE_SOURCE_DIR}/src/utils/config_loader.cpp
)

# Add test executables
add_executable(gnc_tests
  ${GNC_SOURCES}
  sensors_test.cpp
  estimation_test.cpp
  control_test.cpp
  math_utils_test.cpp
  integration_test.cpp
)

# Link libraries
target_include_directories(gnc_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gnc_tests PRIVATE
  GTest::gtest
  GTest::gtest_main
  Eigen3::Eigen
  yaml-cpp
)

# Enable testing
enable_testing()
add_test(NAME GNCTests COMMAND gnc_tests)
