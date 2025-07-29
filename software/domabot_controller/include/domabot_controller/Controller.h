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
    using HoldingRegisters = std::unordered_map<REG_HLD, uint16_t>;
    using InputRegisters   = std::unordered_map<REG_INP, uint16_t>;

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
    template <typename REG> void validateRegisterRange(
      const REG startAddress, const std::size_t cnt
    ) try {
      if (startAddress < REG::START) {
        throw Exception::createError("Invalid start address!");
      }
      const uint8_t maxRegister = (uint8_t) startAddress + cnt;
      if (maxRegister > (uint8_t) REG::END) {
        throw Exception::createError("Exceed last register number!");
      }
    } defaultCatch

    bool readCoil(const COIL address);
    void writeCoil(const COIL address, const bool value);
    void writeCoils(
      const COIL startAddress, const std::vector<bool>& values);
    uint16_t readInputRegister(const REG_INP address);
    InputRegisters readInputRegisters(
      const REG_INP startAddress, const std::size_t cnt);
    uint16_t readHoldingRegister(const REG_HLD address);
    HoldingRegisters readHoldingRegisters(
      const REG_HLD startAddress, const std::size_t cnt);
    void writeHoldingRegister(const REG_HLD address, const uint16_t value);
    void writeHoldingRegisters(
      const REG_HLD startAddress, const std::vector<uint16_t> values);

    static const std::string& getCommandName(const CMD command);
    static const std::string& getStatusName(const STS status);

    void checkStatus(const STS status) const;
    static void checkMode(const MODE mode);
    static void checkDirection(const DIR direction);

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

    void brakeSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
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
        const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
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