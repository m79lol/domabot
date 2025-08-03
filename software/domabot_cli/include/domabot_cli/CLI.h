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

class CLI : public rclcpp::Node {
  public:
    using Ptr = std::shared_ptr<CLI>;
    using CnstPtr = std::shared_ptr<const CLI>;

  protected:
    rclcpp::Subscription<domabot_interfaces::msg::Status>::SharedPtr m_subStatus = nullptr;

    rclcpp::Client<domabot_interfaces::srv::Brake>::SharedPtr m_clientBrake               = nullptr;
    rclcpp::Client<domabot_interfaces::srv::GetData>::SharedPtr m_clientGetData           = nullptr;
    rclcpp::Client<domabot_interfaces::srv::Move>::SharedPtr m_clientMove                 = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SaveSettings>::SharedPtr m_clientSaveSettings = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetDirection>::SharedPtr m_clientSetDirection = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetMode>::SharedPtr m_clientSetMode           = nullptr;
    rclcpp::Client<domabot_interfaces::srv::SetSettings>::SharedPtr m_clientSetSettings   = nullptr;
    rclcpp::Client<domabot_interfaces::srv::Stop>::SharedPtr m_clientStop                 = nullptr;

    domabot_interfaces::msg::Status m_msgLastStatus;
    std::mutex m_mtxLastStatus;

    void statusCallback(const domabot_interfaces::msg::Status& msg);

  public:
    CLI();

    CLI(const CLI& other)            = delete;
    CLI(CLI&& other)                 = delete;
    CLI& operator=(const CLI& other) = delete;
    CLI& operator=(CLI&& other)      = delete;

    virtual ~CLI() = default;

    void runCLI();

}; // CLI

} // Domabot

#endif // Domabot_Controller_h