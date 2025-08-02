#include <domabot_controller/Controller.h>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char * argv[]) try {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  const Domabot::Controller::Ptr node = std::make_shared<Domabot::Controller>();

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}