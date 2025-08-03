#ifndef Domabot_CLI_h
#define Domabot_CLI_h

#include <rclcpp/rclcpp.hpp>

namespace Domabot {

class CLI : public rclcpp::Node {
public:
  using Ptr = std::shared_ptr<CLI>;
  using CnstPtr = std::shared_ptr<const CLI>;

public:
  CLI();

  CLI(const CLI& other)            = delete;
  CLI(CLI&& other)                 = delete;
  CLI& operator=(const CLI& other) = delete;
  CLI& operator=(CLI&& other)      = delete;

  virtual ~CLI() = default;

}; // CLI

} // Domabot

#endif // Domabot_Controller_h