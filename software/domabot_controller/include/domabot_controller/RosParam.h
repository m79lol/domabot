/**
 * @file RosParam.h
 * @brief Domabot Modbus class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CONTROLLER__ROSPARAM_H_
#define DOMABOT_CONTROLLER__ROSPARAM_H_

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>

namespace Domabot {

/**
 * @brief Template class for obtain ROS param value from ROS environment.
 * @tparam T ROS parameter type.
 */
template<typename T> class RosParam {
  public:
    using CnstPtr = std::shared_ptr<const RosParam>;
    using Ptr = std::shared_ptr<RosParam>;

  protected:
    rclcpp::Node& m_node;  ///< Ref to ROS node.
    const std::string m_name;  ///< ROS parameter name

    /**
     * @brief Function to validate parameter value.
     * @throws std::exception on validation fails.
     */
    const std::function<T (const rclcpp::Parameter&)> m_validate;

  public:
    /**
     * @brief Main parameters constructor.
     * @details Creates parameter description and declare it in ROS.
     * @param[in] node Ref to ROS node.
     * @param[in] name Parameter's name
     * @param[in] defaultValue Default value, uses if parameter will be empty.
     * @param[in] validate Validate function for parameter value.
     * @param[in] description Printable parameter's description for logs.
     */
    RosParam(
      rclcpp::Node& node,
      const std::string name,
      const T defaultValue,
      const std::function<T (const rclcpp::Parameter&)> validate,
      const std::string description = ""
    ) try : m_node(node), m_name(name), m_validate(validate) {
      auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
      descriptor.description = description;
      descriptor.read_only = true;

      m_node.declare_parameter(m_name, defaultValue, descriptor);
    } defaultCatch;

    virtual ~RosParam() = default;

    /**
     * @brief Get parameter value from ROS.
     * @details Get parameter and validate it by validate function.
     * @return Parameter value of assigned type.
     * @throws re-throw validate function exception.
     */
    T getValue() const try {
      const auto parameter = m_node.get_parameter(m_name);
      try {
        return m_validate(parameter);
      } catch (const std::exception& e) {
        throw Exception::BackTrackMsg(e,
          Exception::createMsg("Invalid parameter: ", m_name));
      }
    } defaultCatch
};

}  // namespace Domabot

#endif  // DOMABOT_CONTROLLER__ROSPARAM_H_
