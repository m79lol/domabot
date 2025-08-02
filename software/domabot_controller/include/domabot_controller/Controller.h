#ifndef Domabot_Controller_h
#define Domabot_Controller_h

#include <domabot_controller/Exception.h>
#include <domabot_controller/Modbus.h>

#include <domabot_interfaces/msg/status.hpp>

#include <domabot_interfaces/srv/brake.hpp>
#include <domabot_interfaces/srv/get_data.hpp>
#include <domabot_interfaces/srv/move.hpp>
#include <domabot_interfaces/srv/save_settings.hpp>
#include <domabot_interfaces/srv/set_direction.hpp>
#include <domabot_interfaces/srv/set_mode.hpp>
#include <domabot_interfaces/srv/set_settings.hpp>
#include <domabot_interfaces/srv/stop.hpp>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

namespace Domabot {

class Controller : public rclcpp::Node {
  public:
    using Ptr = std::shared_ptr<Controller>;
    using CnstPtr = std::shared_ptr<const Controller>;
  protected:
    Modbus::Ptr m_modbus = nullptr;

    rclcpp::Publisher<domabot_interfaces::msg::Status>::SharedPtr m_pubStatus = nullptr;

    rclcpp::Service<domabot_interfaces::srv::Brake>::SharedPtr m_srvBrake = nullptr;
    rclcpp::Service<domabot_interfaces::srv::GetData>::SharedPtr m_srvGetData = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Move>::SharedPtr m_srvMove = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SaveSettings>::SharedPtr m_srvSaveSettings = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetDirection>::SharedPtr m_srvSetDirection = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetMode>::SharedPtr m_srvSetMode = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetSettings>::SharedPtr m_srvSetSettings = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Stop>::SharedPtr m_srvStop = nullptr;

    rclcpp::TimerBase::SharedPtr m_statusTimer = nullptr;
    rclcpp::CallbackGroup::SharedPtr m_timerCallbackGroup = nullptr;
    uint16_t m_statusRate = 1;

    bool m_isCommandExecuting = false;
    MODE m_currentMode = MODE::TRG;

    static const std::string& getCommandName(const CMD command);
    static const std::string& getStatusName(const STS status);
    static const std::string& getModeName(const MODE mode);

    void checkStatus(const STS status, const bool selfCheck = false) const;
    static void checkMode(const MODE mode, const bool selfCheck = false);
    static void checkDirection(const DIR direction);
    static void checkStepperStatus(const STPR_STS stepperStatus);
    static void checkCommand(const CMD command);

    void setSettingsToRegisters(
      const domabot_interfaces::msg::ControllerSettings& settings);

    void runCommand(const CMD cmd);

    template <typename Service> void processRequestCommand(
      std::shared_ptr<typename Service::Response> res,
      const CMD command
    ) try {
      runCommand(command);
      res->response_data.is_success = true;
      res->response_data.error_message = "";
    } catch (const std::exception& e) {
      res->response_data.is_success = false;
      const auto msg = Exception::BackTrack(e).what();
      res->response_data.error_message = msg;
      RCLCPP_ERROR_STREAM(get_logger(), msg);
    }

    template <typename Service> void processExceptionCommand(
      std::shared_ptr<typename Service::Response> res,
      const std::exception& e
    ) noexcept {
      res->response_data.is_success = false;
      const auto msg = Exception::BackTrack(e).what();
      res->response_data.error_message = msg;
      RCLCPP_ERROR_STREAM(get_logger(), msg);
    }

    template <typename Container> void setRegister(
        Modbus::HoldingRegistersValues& holdingRegs
      , const REG_HLD address
      , const Container& container
      , const std::string& name = ""
      , const bool validateAboveZero = false
    ) try {
      if (!container.empty()) {
        if (validateAboveZero && 0 == container.front()) {
          throw Exception::createError(name, " must be above zero!");
        }
        holdingRegs.emplace(address, container.front());
      }
    } defaultCatch

    void restartStatusTimer(const uint16_t rate);

    void brakeSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Brake::Response> res);
    void getDataSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::GetData::Request> req
      , std::shared_ptr<domabot_interfaces::srv::GetData::Response> res);
    void moveSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Move::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Move::Response> res);
    void saveSettingsSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SaveSettings::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SaveSettings::Response> res);
    void setDirectionSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetDirection::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetDirection::Response> res);
    void setModeSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetMode::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetMode::Response> res);
    void setSettingsSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetSettings::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetSettings::Response> res);
    void stopSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res);

    void statusTimerCallback();

  public:
    Controller();

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() = default;

}; // Controller

} // Domabot

#endif // Domabot_Controller_h