#ifndef Domabot_Controller_h
#define Domabot_Controller_h

#include <domabot_interfaces/msg/status.hpp>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <mutex>

namespace Domabot {

class Controller : public rclcpp::Node {
  public:
    using CnstPtr = std::shared_ptr<const Controller>;
    using Ptr = std::shared_ptr<Controller>;

  protected:
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;
    bool m_isConnected = false;

    rclcpp::Publisher<domabot_interfaces::msg::Status>::SharedPtr m_statusPublisher;
    rclcpp::TimerBase::SharedPtr m_statusTimer;

    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

    // modbus operations
    bool readCoil(const COIL address);
    void writeCoil(const COIL address, const bool value);
    uint16_t readInputRegister(const REG_INP address);
    std::vector<uint16_t> readInputRegisters(const REG_INP address, const std::size_t cnt);
    uint16_t readHoldingRegister(const REG_HLD address);
    void writeHoldingRegister(const REG_HLD address, const uint16_t value);


    void statusTimerCallback();

    void getControllerData();

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