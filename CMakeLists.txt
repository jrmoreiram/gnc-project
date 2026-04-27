cmake_minimum_required(VERSION 3.16)
project(gnc_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(gnc_app
  src/main.cpp
  src/gnc_system.cpp
  src/sensors/imu.cpp
  src/sensors/gps.cpp
  src/sensors/barometer.cpp
  src/estimation/ekf.cpp
  src/estimation/state_estimator.cpp
  src/guidance/guidance_law.cpp
  src/guidance/trajectory_planner.cpp
  src/control/pid_controller.cpp
  src/control/actuator_controller.cpp
  src/control/rate_controller.cpp
  src/flight_loop/flight_loop.cpp
  src/utils/math_utils.cpp
  src/utils/data_logger.cpp
  src/utils/config_loader.cpp
)

target_include_directories(gnc_app PRIVATE include)
target_link_libraries(gnc_app PRIVATE Eigen3::Eigen yaml-cpp)
