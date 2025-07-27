#ifndef Domabot_Controller_h
#define Domabot_Controller_h

#include <domabot_controller/Exception.h>

#include <domabot_interfaces/msg/status.hpp>

#include <domabot_interfaces/srv/brake.hpp>
#include <domabot_interfaces/srv/get_data.hpp>
#include <domabot_interfaces/srv/move.hpp>
#include <domabot_interfaces/srv/set_direction.hpp>
#include <domabot_interfaces/srv/set_mode.hpp>
#include <domabot_interfaces/srv/set_settings.hpp>
#include <domabot_interfaces/srv/stop.hpp>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <mutex>

namespace Domabot {

class Controller : public rclcpp::Node {
  protected:
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;
    bool m_isConnected = false;

    rclcpp::Publisher<domabot_interfaces::msg::Status>::SharedPtr m_pubStatus = nullptr;

    rclcpp::Service<domabot_interfaces::srv::Brake>::SharedPtr m_srvBrake = nullptr;
    rclcpp::Service<domabot_interfaces::srv::GetData>::SharedPtr m_srvGetData = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Move>::SharedPtr m_srvMove = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetDirection>::SharedPtr m_srvSetDirection = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetMode>::SharedPtr m_srvSetMode = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetSettings>::SharedPtr m_srvSetSettings = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Stop>::SharedPtr m_srvStop = nullptr;

    rclcpp::TimerBase::SharedPtr m_statusTimer = nullptr;

    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

    // modbus operations
    bool readCoil(const COIL address);
    void writeCoil(const COIL address, const bool value);
    void writeCoils(
      const COIL startAddress, const std::vector<bool>& values);
    uint16_t readInputRegister(const REG_INP address);
    std::vector<uint16_t> readInputRegisters(
      const REG_INP startAddress, const std::size_t cnt);
    uint16_t readHoldingRegister(const REG_HLD address);
    void writeHoldingRegister(const REG_HLD address, const uint16_t value);

    static const std::string& getCommandName(const CMD command);
    static const std::string& getStatusName(const STS status);

    void checkStatus(const STS status) const;
    static void checkMode(const MODE mode);

    STS runCommand(const CMD cmd);

    template <typename Service> void processRequestCommand(
      std::shared_ptr<typename Service::Response> res,
      const CMD command
    ) try {
      res->controller_status = (uint8_t) STS::ERR_UNKNOWN;
      const STS status = runCommand(command);
      res->controller_status = (uint8_t) status;
      checkStatus(status);
      res->response_data.is_success = true;
      res->response_data.error_message = "";
    } defaultCatch

    template <typename Service> void processExceptionCommand(
      std::shared_ptr<typename Service::Response> res,
      const std::exception& e
    ) noexcept {
      res->response_data.is_success = false;
      const auto msg = Exception::BackTrack(e).what();
      res->response_data.error_message = msg;
      RCLCPP_ERROR_STREAM(get_logger(), msg);
    }

    void brakeSrvCallback(
        [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Brake::Response> res);
    void getDataSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::GetData::Request> req
      , std::shared_ptr<domabot_interfaces::srv::GetData::Response> res);
    void moveSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Move::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Move::Response> res);
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
        [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res);

    void statusTimerCallback();

  public:
    Controller();

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() noexcept;



}; // Controller

} // Domabot

#endif // Domabot_Controller_h