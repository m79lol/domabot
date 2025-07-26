#ifndef Domabot_ControllerParams_h
#define Domabot_ControllerParams_h

#include <domabot_controller/Exception.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

namespace Domabot {

class ControllerParams {
  protected:
    template <typename T, std::size_t K>
    static T checkInArray(const std::array<T, K>& allowed, const T value) try {
      const auto it = std::find(std::begin(allowed), std::end(allowed), value);
      if (it != allowed.end()) {
        return value;
      }

      const auto concat = [](std::string a, const int b) {
        return std::move(a) + ", " + std::to_string(b);
      };
      const std::string allowedStr = std::accumulate(
          std::next(allowed.rbegin()), allowed.rend(), std::to_string(allowed[0])
        , concat);

      throw Exception::createError("The value ", value, " is not within allowed values: ", allowedStr);
    } defaultCatch

    static std::string checkIsEmpty(const rclcpp::Parameter& parameter);

  public:
    ControllerParams() = delete;

    static unsigned int getBaudRate(rclcpp::Node& node);
    static unsigned int getDataBits(rclcpp::Node& node);
    static char         getParity  (rclcpp::Node& node);
    static std::string  getPath    (rclcpp::Node& node);
    static unsigned int getStopBits(rclcpp::Node& node);
    static unsigned int getSlaveId (rclcpp::Node& node);

}; // ControllerParams

} // Domabot

#endif // Domabot_ControllerParams_h