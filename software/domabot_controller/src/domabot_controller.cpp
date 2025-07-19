#include <domabot_controller/Controller.h>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char * argv[]) try {
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("domabot_controller");
  Domabot::Controller controller(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} catch (const std::exception& e) {
  std::cout << e.what() << std::endl;
  return 1;
}