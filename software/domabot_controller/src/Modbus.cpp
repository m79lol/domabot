/**
 * @file Modbus.cpp
 * @brief Domabot Modbus class source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_controller/Modbus.h>

namespace Domabot {

Modbus::Modbus(
    const rclcpp::Logger& logger
  , const std::string& path
  , const unsigned int baudRate
  , const char parity
  , const unsigned int dataBits
  , const unsigned int stopBits
  , const unsigned int slaveId
  , const double connectDelay
  , const double modbusTimeout
  , const std::function<void()>& connectCallback
) try : m_logger(logger), m_connectDelay(connectDelay), m_connectCallback(connectCallback) {
  RCLCPP_DEBUG_STREAM(m_logger, "Modbus context create...");
  m_cntx = modbus_new_rtu(
    path.c_str(), baudRate, parity, dataBits, stopBits);

  if (NULL == m_cntx) {
    throw Exception::createError("Unable to create the libmodbus context!");
  }
  RCLCPP_DEBUG_STREAM(m_logger, "...ok");

  /*constexpr int debugFlag = 1;
  RCLCPP_DEBUG_STREAM(m_logger, "Modbus set debug " << debugFlag <<  "...");
  if (modbus_set_debug(m_cntx, debugFlag) < 0) {
    throw Exception::createError(
      "Set modbus set debug error: ", modbus_strerror(errno));
  }
  RCLCPP_DEBUG_STREAM(m_logger, "...ok");*/

  RCLCPP_DEBUG_STREAM(m_logger, "Modbus set slave id " << slaveId <<  "...");
  if (modbus_set_slave(m_cntx, slaveId) < 0) {
    throw Exception::createError(
      "Set modbus slave address error: ", modbus_strerror(errno));
  }
  RCLCPP_DEBUG_STREAM(m_logger, "...ok");

  constexpr uint32_t second =  1'000'000L;
  const uint32_t timeoutMicroSecs = static_cast<uint32_t>(
    modbusTimeout * 1'000'000.0f);
  const uint32_t responseSec = timeoutMicroSecs / second;
  const uint32_t responseUSec = timeoutMicroSecs % second;
  RCLCPP_DEBUG_STREAM(
      m_logger
    , "Modbus set timeout to " << responseSec <<  " sec "
      <<  responseUSec << " usec.");
  if (modbus_set_response_timeout(m_cntx, responseSec, responseUSec) < 0) {
    throw Exception::createError(
      "Set modbus time out error: ", modbus_strerror(errno));
  }
  RCLCPP_DEBUG_STREAM(m_logger, "...ok");
} catch (const std::exception& e) {
  free();
  throw Exception::BackTrack(e);
}

void Modbus::free() noexcept {
  if (nullptr != m_cntx) {
    const std::lock_guard<std::mutex> lock(m_mtx);
    modbus_close(m_cntx);
    modbus_free(m_cntx);
    m_cntx = nullptr;
  }
}

Modbus::~Modbus() noexcept {
  free();
}

void Modbus::runModbusOperation(
  std::function<bool (modbus_t*)> operation
) try {
  const std::lock_guard<std::mutex> lock(m_mtx);

  unsigned int attempts = 2;
  while(attempts--) {
    try {
      if (!m_isConnected) {
        RCLCPP_INFO_STREAM(m_logger, "Trying to re-establish link...");
        if (modbus_connect(m_cntx) < 0) {
          throw Exception::createError(
            "Could not establish link. Modbus error: ", modbus_strerror(errno));
        }
        rclcpp::Rate(m_connectDelay).sleep();
        m_isConnected = true;
        RCLCPP_INFO_STREAM(m_logger, "Reconnected.");
        if (m_connectCallback) {
          m_connectCallback();
        }
      }
      if (!operation(m_cntx)) {
        const std::string error = modbus_strerror(errno);
        if (m_isConnected) {
          modbus_close(m_cntx);
          m_isConnected = false;
        }
        throw Exception::createError(
            "Modbus operation failed. Modbus error: "
          , error);
      }
      break;
    } catch (const std::exception& e) {
      if (attempts) {
        RCLCPP_WARN_STREAM(m_logger, e.what());
        continue;
      }
      throw;
    }
  }
  RCLCPP_DEBUG_STREAM(m_logger, "Modbus operation successful.");
} defaultCatch

bool Modbus::readCoil(const COIL address) try {
  return readCoils({address}).at(address);
} defaultCatch

Modbus::CoilsValues Modbus::readCoils(
  const Coils& coils
) try {
  if (coils.empty()) { return {}; }

  const ReadFunc<bool> readFunc = [this](
    const uint8_t startAddress, const size_t cnt
  ) {
    std::vector<uint8_t> tmp(cnt, 0);
    runModbusOperation([&tmp, &startAddress](modbus_t* cntx){
      return static_cast<int>(tmp.size()) == modbus_read_bits(
          cntx
        , static_cast<int>(startAddress)
        , static_cast<int>(tmp.size())
        , tmp.data());
    });

    std::vector<bool> coils(cnt, false);
    for (size_t i = 0; i < cnt; ++i) {
      coils[i] = 0 != tmp[i];
    }
    return coils;
  };

  return readItems<COIL, bool>(coils, readFunc);
} defaultCatch

void Modbus::writeCoil(const COIL address, const bool value) try {
  return writeCoils({{address, value}});
} defaultCatch

void Modbus::writeCoils(
  const CoilsValues& coilValues
) try {
  if (coilValues.empty()) { return; }

  const WriteFunc<bool> writeFunc = [this](
    const uint8_t startAddress, const std::vector<bool>& values
  ) {
    runModbusOperation([&startAddress, &values](modbus_t* cntx){
      std::vector<uint8_t> coils(values.size(), 0);
      for (size_t i = 0; i < values.size(); ++i) {
        coils[i] = values[i] ? 1 : 0;
      }

      return static_cast<int>(coils.size()) == modbus_write_bits(
          cntx
        , static_cast<int>(startAddress)
        , static_cast<int>(coils.size())
        , coils.data());
    });
  };

  writeItems<COIL, bool>(coilValues, writeFunc);
} defaultCatch

uint16_t Modbus::readInputRegister(const REG_INP address) try {
  return readInputRegisters({address}).at(address);
} defaultCatch

Modbus::InputRegistersValues Modbus::readInputRegisters(
  const InputRegisters& registers
) try {
  if (registers.empty()) { return {}; }

  const ReadFunc<uint16_t> readFunc = [this](
    const uint8_t startAddress, const size_t cnt
  ) {
    std::vector<uint16_t> tmp(cnt, 0);
    runModbusOperation([&tmp, &startAddress](modbus_t* cntx){
      return static_cast<int>(tmp.size()) == modbus_read_input_registers(
          cntx
        , static_cast<int>(startAddress)
        , static_cast<int>(tmp.size())
        , tmp.data());
    });
    return tmp;
  };

  return readItems<REG_INP, uint16_t>(registers, readFunc);
} defaultCatch

uint16_t Modbus::readHoldingRegister(const REG_HLD address) try {
  return readHoldingRegisters({address}).at(address);
} defaultCatch

Modbus::HoldingRegistersValues Modbus::readHoldingRegisters(
  const HoldingRegisters& registers
) try {
  if (registers.empty()) { return {}; }

  const ReadFunc<uint16_t> readFunc = [this](
    const uint8_t startAddress, const size_t cnt
  ) {
    std::vector<uint16_t> tmp(cnt, 0);
    runModbusOperation([&tmp, &startAddress](modbus_t* cntx){
      return static_cast<int>(tmp.size()) == modbus_read_registers(
          cntx
        , static_cast<int>(startAddress)
        , static_cast<int>(tmp.size())
        , tmp.data());
    });
    return tmp;
  };

  return readItems<REG_HLD, uint16_t>(registers, readFunc);
} defaultCatch

void Modbus::writeHoldingRegister(
  const REG_HLD address, const uint16_t value
) try {
  writeHoldingRegisters({{address, value}});
} defaultCatch

void Modbus::writeHoldingRegisters(
  const HoldingRegistersValues& registerValues
) try {
  if (registerValues.empty()) { return; }

  const WriteFunc<uint16_t> writeFunc = [this](
    const uint8_t startAddress, const std::vector<uint16_t>& values
  ) {
    runModbusOperation([&startAddress, &values](modbus_t* cntx) {
      return static_cast<int>(values.size()) == modbus_write_registers(
          cntx
        , static_cast<int>(startAddress)
        , static_cast<int>(values.size())
        , values.data());
    });
  };

  writeItems<REG_HLD, uint16_t>(registerValues, writeFunc);
} defaultCatch

}  // namespace Domabot
