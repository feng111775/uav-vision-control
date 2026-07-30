cmake_minimum_required(VERSION 3.8)
project(car_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(car_motion_smoke_node src/sim/car_motion_smoke_node.cpp)
ament_target_dependencies(car_motion_smoke_node rclcpp geometry_msgs nav_msgs)
add_library(car_control_sim
  src/sim/d_task_track_geometry.cpp
  src/sim/virtual_gray_sensor.cpp
)
target_include_directories(car_control_sim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(car_control_sim PUBLIC car_control_core)
add_executable(virtual_gray_sensor_node src/sim/virtual_gray_sensor_node.cpp)
ament_target_dependencies(virtual_gray_sensor_node rclcpp nav_msgs std_msgs)
target_link_libraries(virtual_gray_sensor_node car_control_sim)
target_include_directories(car_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS car_control_health car_control_core car_control_sim
  EXPORT export_car_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS car_motion_smoke_node virtual_gray_sensor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config urdf worlds firmware
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(car_control_launch PACKAGE_DIR python/car_control_launch)

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

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME car_control_sim_static_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_car_sim_static.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_executable(test_virtual_gray_sensor test/test_virtual_gray_sensor.cpp)
  target_include_directories(test_virtual_gray_sensor PRIVATE test)
  target_link_libraries(test_virtual_gray_sensor PRIVATE car_control_sim)
  add_test(NAME car_control_virtual_gray_sensor_test COMMAND test_virtual_gray_sensor)
endif()

ament_export_targets(export_car_control HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
