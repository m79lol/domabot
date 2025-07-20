#ifndef Domabot_RosParam_h
#define Domabot_RosParam_h

#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace Domabot {

template<typename T> class RosParam {
  public:
    using CnstPtr = std::shared_ptr<const RosParam>;
    using Ptr = std::shared_ptr<RosParam>;

  protected:
    const rclcpp::Node::SharedPtr m_node;
    const std::string m_name;
    const std::function<T (const rclcpp::Parameter&)> m_validate;

  public:
    RosParam(
      const rclcpp::Node::SharedPtr node,
      const std::string name,
      const T defaultValue,
      const std::function<T (const rclcpp::Parameter&)> validate,
      const std::string description = ""
    ) try : m_node(node), m_name(name), m_validate(validate) {
      auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
      descriptor.description = description;

      m_node->declare_parameter(m_name, defaultValue, descriptor);
    } defaultCatch;

    virtual ~RosParam() = default;

    T getValue() const try {
      const auto parameter = m_node->get_parameter(m_name);
      try {
        return m_validate(parameter);
      } catch (const std::exception& e) {
        throw Exception::BackTrackMsg(e,
          Exception::createMsg("Invalid parameter: ", m_name));
      };
    } defaultCatch

};

} // Domabot

#endif // Domabot_RosParam_h