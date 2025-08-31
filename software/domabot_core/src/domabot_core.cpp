/**
 * @file domabot_core.cpp
 * @brief Domabot Core Node main source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_core/Core.h>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char* argv[]) try {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  const Domabot::Core::Ptr node = std::make_shared<Domabot::Core>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}
