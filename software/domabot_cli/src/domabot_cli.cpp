/**
 * @file domabot_controller.cpp
 * @brief Domabot Console Line Interface Node main source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_cli/CLI.h>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char * argv[]) try {
  rclcpp::init(argc, argv);

  const Domabot::CLI::Ptr node = std::make_shared<Domabot::CLI>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  std::thread spinThread([&executor]() {
    executor.spin();
  });

  int exitCode = 0;
  try {
    node->runCLI();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    exitCode = 1;
  }
  rclcpp::shutdown();
  spinThread.join();

  return exitCode;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}
