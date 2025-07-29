cmake_minimum_required(VERSION 3.8)
project(domabot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(domabot_interfaces REQUIRED)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/Controller.cpp
  src/ControllerParams.cpp
  src/Exception.cpp
  src/Modbus.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp domabot_interfaces)

include_directories(
  include
  "../../firmware/include"
  "/usr/include/modbus"
)

target_link_libraries(${PROJECT_NAME}
  modbus
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
