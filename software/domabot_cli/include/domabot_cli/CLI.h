/**
 * @file
 * @brief Domabot CLI node class header file.
*/
#ifndef Domabot_CLI_h
#define Domabot_CLI_h

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

#include <mutex>

namespace Domabot {

/**
 * @brief Domabot CLI Node class.
 *
 * @details Inherits Node ROS class, implements ROS node and process both
 * ROS & CLI (command line interface) interfaces.
  */
class CLI : public rclcpp::Node {
  public: // pointer's short cuts
    using Ptr = std::shared_ptr<CLI>;
    using CnstPtr = std::shared_ptr<const CLI>;

  protected:
    // topic subscribers for domabot controller node
    rclcpp::Subscription<domabot_interfaces::msg::Status>::SharedPtr m_subStatus = nullptr;

    // service clients for domabot controller node
    rclcpp::Client<domabot_interfaces::srv::Brake>::SharedPtr m_clientBrake               = nullptr;
    rclcpp::Client<domabot_interfaces::srv::GetData>::SharedPtr m_clientGetData           = nullptr;
    rclcpp::Client<domabot_interfaces::srv::Move>::SharedPtr m_clientMove                 = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SaveSettings>::SharedPtr m_clientSaveSettings = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetDirection>::SharedPtr m_clientSetDirection = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetMode>::SharedPtr m_clientSetMode           = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetSettings>::SharedPtr m_clientSetSettings   = nullptr;
    rclcpp::Client<domabot_interfaces::srv::Stop>::SharedPtr m_clientStop                 = nullptr;

    domabot_interfaces::msg::Status m_msgLastStatus; ///< To transfer msg between subscriber and main threads
    std::mutex m_mtxLastStatus;

    /**
     * @brief Receive status message callback.
     * @details Calls on every message received by subscriber.
     *
     * @param[in] msg Controller status message.
     */
    void statusCallback(const domabot_interfaces::msg::Status& msg);

  public:
    CLI();

    CLI(const CLI& other)            = delete;
    CLI(CLI&& other)                 = delete;
    CLI& operator=(const CLI& other) = delete;
    CLI& operator=(CLI&& other)      = delete;

    virtual ~CLI() = default;

    /**
     * @brief Main method for processing command line interface.
     */
    void runCLI();

}; // CLI

} // Domabot

#endif // Domabot_Controller_h
