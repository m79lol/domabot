/**
 * @file ControllerParams.h
 * @brief Domabot Controller Params header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CONTROLLER__CONTROLLERPARAMS_H_
#define DOMABOT_CONTROLLER__CONTROLLERPARAMS_H_

#include <domabot_common_lib/Exception.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>
#include <utility>

namespace Domabot {

/**
 * @brief Utility static class for describe controller node params.
 * @details To safe clean code of controller's class.
 */
class ControllerParams {
  protected:
    /**
     * @brief Find a value in allowed list.
     * @tparam T Type of allowed item.
     * @tparam K Size of array with allowed items.
     * @param[in] allowed Predefined size array with allowed values.
     * @param[in] value Validating value.
     * @return Validated value (architecture work around).
     * @throws if value not in allowed values list.
     */
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

      throw Exception::createError("The value ", value,
        " is not within allowed values: ", allowedStr);
    } defaultCatch

    /**
     * @brief Checks that string parameter is not empty string.
     * @param[in] parameter Ref to ROS string parameter.
     * @return Obtained value from ROS parameter.
     * @throws if parameter return empty string.
     */
    static std::string checkIsEmpty(const rclcpp::Parameter& parameter);

  public:
    /**
     * @brief This is full static class. No instances.
     */
    ControllerParams() = delete;

    /**
     * @brief Get baud rate speed for RTU communication from ROS parameters.
     * @details Validate obtained speed for allowed list:
     * 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400.
     * Default is 9600.
     * @param[in] node Pointer to current node instance.
     * @return Obtained baud rate.
     */
    static unsigned int getBaudRate(rclcpp::Node& node);

    /**
     * @brief Get data bits for RTU communication from ROS parameters.
     * @details Validate obtained value for allowed list: [5,6,7,8].
     * Default is 8.
     * @param[in] node Pointer to current node instance.
     * @return Obtained data bits.
     */
    static unsigned int getDataBits(rclcpp::Node& node);

    /**
     * @brief Get parity for RTU communication from ROS parameters.
     * @details Validate obtained value for allowed list: [N,E,O].
     * Default is N.
     * @param[in] node Pointer to current node instance.
     * @return Obtained parity.
     */
    static char         getParity  (rclcpp::Node& node);

    /**
     * @brief Get path to RTU serial socket from ROS parameters.
     * @details It's usually like /dev/ttyUSB0
     * @param[in] node Pointer to current node instance.
     * @return Obtained path as string.
     */
    static std::string  getPath    (rclcpp::Node& node);

    /**
     * @brief Get stop bits for RTU communication from ROS parameters.
     * @details Validate obtained value for allowed list: [1,2].
     * Default is 1.
     * @param[in] node Pointer to current node instance.
     * @return Obtained stop bits.
     */
    static unsigned int getStopBits(rclcpp::Node& node);

    /**
     * @brief Get controller's modbus slave address (id) from ROS parameters.
     * @details That modbus device address within modbus network.
     * Default is 1.
     * @param[in] node Pointer to current node instance.
     * @return Obtained slave id.
     */
    static unsigned int getSlaveId (rclcpp::Node& node);
};  // ControllerParams

}  // namespace Domabot

#endif  // DOMABOT_CONTROLLER__CONTROLLERPARAMS_H_
