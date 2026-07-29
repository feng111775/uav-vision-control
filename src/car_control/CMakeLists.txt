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

install(
  TARGETS car_control_health
  EXPORT export_car_control_health
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
endif()

ament_export_targets(export_car_control_health HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
