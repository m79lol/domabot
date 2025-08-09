/**
 * @file CLI.h
 * @brief Domabot CLI node class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CLI__CLI_H_
#define DOMABOT_CLI__CLI_H_

#include <domabot_common_lib/Exception.h>

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
