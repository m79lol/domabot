#ifndef Domabot_Modbus_h
#define Domabot_Modbus_h

#include <domabot_controller/Exception.h>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace Domabot {

class Modbus {
  public:
    using Ptr = std::shared_ptr<Modbus>;
    using CnstPtr = std::shared_ptr<const Modbus>;
    using HoldingRegisters = std::unordered_map<REG_HLD, uint16_t>;
    using InputRegisters   = std::unordered_map<REG_INP, uint16_t>;

  protected:
    const rclcpp::Logger m_logger;
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;
    bool m_isConnected = false;

    void free() noexcept;

    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

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

  public:
    Modbus(
      const rclcpp::Logger& logger
      , const std::string& path
      , const unsigned int baudRate
      , const char parity
      , const unsigned int dataBits
      , const unsigned int stopBits
      , const unsigned int slaveId
    );

    Modbus(const Modbus& other)            = delete;
    Modbus(Modbus&& other)                 = delete;
    Modbus& operator=(const Modbus& other) = delete;
    Modbus& operator=(Modbus&& other)      = delete;

    virtual ~Modbus() noexcept;

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

}; // Modbus

} // Domabot

#endif // Domabot_Modbus_h