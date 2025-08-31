cmake_minimum_required(VERSION 3.10)
project(domabot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

option(ENABLE_CPPCHECK "Enable cppcheck tool for static code analysis." OFF)

if(ENABLE_CPPCHECK)
  set(CMAKE_CXX_CPPCHECK
    cppcheck
    --enable=warning,style,performance,portability,information,unusedFunction
    --language=c++
    --std=c++17
    --inline-suppr
    --inconclusive
    --max-ctu-depth=10
    --suppressions-list=${CMAKE_CURRENT_SOURCE_DIR}/cppcheck_suppress_list.txt
  )
  message(STATUS "cppcheck enabled.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(domabot_controller_interfaces REQUIRED)
find_package(domabot_common_lib REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp
  src/Core.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  domabot_controller_interfaces
  domabot_common_lib
)

include_directories(
  include
  ${domabot_common_lib_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  domabot_common_lib::domabot_common_lib
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
