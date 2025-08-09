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
  executor.add_node(node);

  std::thread spinThread([&executor]() {
      executor.spin();
  });

  node->runCLI();

  spinThread.join();
  rclcpp::shutdown();
  return 0;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}
