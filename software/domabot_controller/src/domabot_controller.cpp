#include <domabot_controller/Controller.h>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char * argv[]) try {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Domabot::Controller>());
  rclcpp::shutdown();
  return 0;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}