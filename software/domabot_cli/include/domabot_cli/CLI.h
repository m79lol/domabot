/**
 * @file CLI.h
 * @brief Domabot CLI node class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CLI__CLI_H_
#define DOMABOT_CLI__CLI_H_

#include <domabot_cli/StringTools.h>
#include <domabot_cli/UserInteraction.h>

#include <domabot_common_lib/Exception.h>
#include <domabot_common_lib/RosService.h>

#include <domabot_interfaces/msg/status.hpp>
#include <domabot_interfaces/srv/brake.hpp>
#include <domabot_interfaces/srv/get_data.hpp>
#include <domabot_interfaces/srv/move.hpp>
#include <domabot_interfaces/srv/save_settings.hpp>
#include <domabot_interfaces/srv/set_direction.hpp>
#include <domabot_interfaces/srv/set_mode.hpp>
#include <domabot_interfaces/srv/set_settings.hpp>
#include <domabot_interfaces/srv/stop.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>

namespace Domabot {

/**
 * @brief Domabot CLI Node class.
 * @details Inherits Node ROS class, implements ROS node and process both
 * ROS & CLI (command line interface) interfaces.
  */
class CLI : public rclcpp::Node {
  public:  // pointer's short cuts
    using Ptr = std::shared_ptr<CLI>;
    using CnstPtr = std::shared_ptr<const CLI>;

  protected:
    #define DI domabot_interfaces

    // topic subscribers for domabot controller node
    rclcpp::Subscription<DI::msg::Status>::SharedPtr m_subStatus = nullptr;

    // service clients for domabot controller node
    rclcpp::Client<DI::srv::Brake>::SharedPtr m_clientBrake               = nullptr;
    rclcpp::Client<DI::srv::GetData>::SharedPtr m_clientGetData           = nullptr;
    rclcpp::Client<DI::srv::Move>::SharedPtr m_clientMove                 = nullptr;
    rclcpp::Client<DI::srv::SaveSettings>::SharedPtr m_clientSaveSettings = nullptr;
    rclcpp::Client<DI::srv::SetDirection>::SharedPtr m_clientSetDirection = nullptr;
    rclcpp::Client<DI::srv::SetMode>::SharedPtr m_clientSetMode           = nullptr;
    rclcpp::Client<DI::srv::SetSettings>::SharedPtr m_clientSetSettings   = nullptr;
    rclcpp::Client<DI::srv::Stop>::SharedPtr m_clientStop                 = nullptr;

    /** @brief Container to transfer msg between subscriber and main threads */
    DI::msg::Status m_msgLastStatus;
    std::mutex m_mtxLastStatus;

    /**
     * @brief Obtain setting value from user via console input.
     * @details Also print current value. User may leave setting change by
     * input 'X' char.
     * @tparam Container Setting container field from ROS msg file.
     * @param[in] settingName Printable setting name.
     * @param[out] destValue Destination setting container. May be empty if user
     * nothing change.
     * @param[in] currentValue Current setting container with actual value for hint.
     * @return True if something setting change.
     * @throws If current setting container is empty.
     */
    template <typename Container> bool inputSettingValue(
        const std::string& settingName
      , Container& destValue
      , const Container& currentValue
    ) try {
      if (currentValue.empty()) {
        throw Exception::createError(
          "Empty current value for setting: ",settingName);
      }

      static const char emptyChar = 'X';
      std::stringstream ss;
      ss << "Input " << settingName << " value. ";
      ss << "Current :" << currentValue.front() << ". ";
      ss << "Input " << emptyChar << " to leave current value." << std::endl;
      ss << "Value: ";

      const std::string entered = UserInteraction::askInput(ss.str());
      if (std::string(&emptyChar) == entered) {
        return false;
      }

      destValue.push_back(
        StringTools::stringToNumber<typename Container::value_type>(entered));
      return true;
    } defaultCatch

    /**
     * @brief Obtain all stepper's settings values from user's input.
     * @details If user nothing change, then leaves empty settings container.
     * @tparam Container Stepper setting container.
     * @param[out] newContainer Destination container. May be empty if user
     * nothing change.
     * @param[in] oldContainer Current stepper settings container with actual values.
     * @return True if something setting change.
     * @throws If current setting container is empty.
     */
    template <typename Container> bool inputStepperSettings(
        Container& newContainer
      , const Container& oldContainer
    ) try {
      if (oldContainer.empty()) {
        throw Exception::createError("Empty old stepper settings!");
      }

      const auto oldSettings = oldContainer.front();
      DI::msg::StepperSettings newSettings;
      bool updated = false;
      updated |= inputSettingValue(
          "target"
        , newSettings.target
        , oldSettings.target);
      updated |= inputSettingValue(
          "max speed"
        , newSettings.max_speed
        , oldSettings.max_speed);
      updated |= inputSettingValue(
          "max acceleration"
        , newSettings.max_acceleration
        , oldSettings.max_acceleration);
      updated |= inputSettingValue(
          "gear ratio"
        , newSettings.gear_ratio
        , oldSettings.gear_ratio);
      updated |= inputSettingValue(
          "wheel diameter"
        , newSettings.wheel_diameter
        , oldSettings.wheel_diameter);
      updated |= inputSettingValue(
          "is forward"
        , newSettings.is_forward
        , oldSettings.is_forward);

      if (!updated) {
        return false;
      }

      newContainer.push_back(newSettings);
      return true;
    } defaultCatch

    /**
     * @brief Obtain all controller's settings values from user's input.
     * @param[in] newSettings Settings destination instance.
     * @return True if something setting change.
     */
    bool inputSettings(DI::msg::ControllerSettings newSettings);

    /**
     * @brief Call controller's service wrapper method.
     * @tparam ServiceType Controller's service type.
     * @param[in] client Service client from current node.
     * @param[in] request Filled service request.
     * @return Service response.
     */
    template <typename ServiceType>
    std::shared_ptr<const typename ServiceType::Response> callService(
        const typename rclcpp::Client<ServiceType>::SharedPtr client
      , const std::shared_ptr<typename ServiceType::Request> request
    ) try {
      return RosService::callAndVerifyService<ServiceType>(
          shared_from_this()
        , get_logger().get_child("service")
        , client
        , request
      );
    } defaultCatch

    /**
     * @brief Receive status message callback.
     * @details Calls on every message received by subscriber.
     * @param[in] msg Controller status message.
     */
    void statusCallback(const DI::msg::Status& msg);

    #undef DI

  public:
    CLI();

    CLI(const CLI& other)            = delete;
    CLI(CLI&& other)                 = delete;
    CLI& operator=(const CLI& other) = delete;
    CLI& operator=(CLI&& other)      = delete;

    virtual ~CLI() = default;

    /** @brief Main method for processing command line interface. */
    void runCLI();
};  // CLI

}  // namespace Domabot

#endif  // DOMABOT_CLI__CLI_H_
