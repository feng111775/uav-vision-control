cmake_minimum_required(VERSION 3.8)
project(car_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(car_control_health src/health.cpp)
target_include_directories(car_control_health
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(car_control_core
  src/core/encoder_velocity_estimator.cpp
  src/core/pi_controller.cpp
  src/core/line_error_estimator.cpp
  src/core/line_pd_controller.cpp
  src/core/speed_planner.cpp
  src/core/line_recovery.cpp
  src/core/button_debouncer.cpp
  src/core/progress_state_machine.cpp
)
target_include_directories(car_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS car_control_health car_control_core
  EXPORT export_car_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config urdf worlds firmware
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_health test/test_health.cpp)
  target_link_libraries(test_health PRIVATE car_control_health)
  add_test(NAME car_control_health_test COMMAND test_health)

  foreach(module
      encoder_velocity
      pi_controller
      line_error
      line_pd
      speed_planner
      line_recovery
      button_debouncer
      progress_state_machine)
    add_executable(test_${module} test/test_${module}.cpp)
    target_include_directories(test_${module} PRIVATE test)
    target_link_libraries(test_${module} PRIVATE car_control_core)
    add_test(NAME car_control_${module}_test COMMAND test_${module})
  endforeach()
endif()

ament_export_targets(export_car_control HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
