cmake_minimum_required(VERSION 3.10)
project(domabot_common_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

option(ENABLE_CPPCHECK "Enable cppcheck tool for static code analysis." OFF)

if(ENABLE_CPPCHECK)
  set(CMAKE_CXX_CPPCHECK
    cppcheck
    --enable=warning,style,performance,portability,information
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

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

include_directories(
  include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
