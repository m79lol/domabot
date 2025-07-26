
#include <domabot_controller/Controller.h>
#include <domabot_controller/ControllerParams.h>
#include <domabot_controller/Exception.h>

namespace Domabot {

Controller::Controller() try : Node("domabot_controller") {

  RCLCPP_DEBUG_STREAM(get_logger(), "Modbus context create...");
  m_cntx = modbus_new_rtu(
    ControllerParams::getPath    (*this).c_str(),
    ControllerParams::getBaudRate(*this),
    ControllerParams::getParity  (*this),
    ControllerParams::getDataBits(*this),
    ControllerParams::getStopBits(*this)
  );
  if (NULL == m_cntx) {
    throw Exception::createError("Unable to create the libmodbus context!");
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "...ok");

  const unsigned int slaveId = ControllerParams::getSlaveId(*this);
  RCLCPP_DEBUG_STREAM(get_logger(), "Modbus set slave id " << slaveId <<  "...");
  if (modbus_set_slave(m_cntx, slaveId) < 0) {
    throw Exception::createError(
      "Set modbus slave address error: ", modbus_strerror(errno));
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "...ok");

  const auto protocolVersion = readInputRegister(REG_INP::VER);
  if (PROTOCOL_VERSION != protocolVersion) {
    throw Exception::createError(
        "Protocol version mismatch! Awaited: ", PROTOCOL_VERSION
      , ". Obtained: ", protocolVersion);
  }

  m_pubStatus = create_publisher<domabot_interfaces::msg::Status>("status", 1);

  m_srvBrake = create_service<domabot_interfaces::srv::Brake>(
    "brake",
    std::bind(
      &Controller::brakeSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvGetData = create_service<domabot_interfaces::srv::GetData>(
    "get_controller",
    std::bind(
      &Controller::getDataSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvMove = create_service<domabot_interfaces::srv::Move>(
    "move",
    std::bind(
      &Controller::moveSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvSetDirection = create_service<domabot_interfaces::srv::SetDirection>(
    "set_direction",
    std::bind(
      &Controller::setDirectionSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvSetMode = create_service<domabot_interfaces::srv::SetMode>(
    "set_mode",
    std::bind(
      &Controller::setModeSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvSetSettings = create_service<domabot_interfaces::srv::SetSettings>(
    "set_settings",
    std::bind(
      &Controller::setSettingsSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  m_srvStop = create_service<domabot_interfaces::srv::Stop>(
    "stop",
    std::bind(
      &Controller::stopSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  const auto pubRate = readHoldingRegister(REG_HLD::RATE);
  const std::chrono::milliseconds pubPeriod(1000 / pubRate);

  m_statusTimer = create_wall_timer(
      pubPeriod, std::bind(&Controller::statusTimerCallback, this));
} defaultCatch

Controller::~Controller() noexcept {
  if (nullptr != m_cntx) {
    const std::lock_guard<std::mutex> lock(m_mtx);
    modbus_close(m_cntx);
    modbus_free(m_cntx);
    m_cntx = nullptr;
  }
}

void Controller::runModbusOperation(
  std::function<bool (modbus_t*)> operation
) try {
  const std::lock_guard<std::mutex> lock(m_mtx);

  unsigned int attempts = 2;
  while(attempts--) {
    try {
      bool isNewConnect = false;
      if (!m_isConnected) {
        if (modbus_connect(m_cntx) < 0) {
          throw Exception::createError(
            "Could not establish link. Modbus error: ", modbus_strerror(errno));
        }
        m_isConnected = true;
        isNewConnect = true;
        RCLCPP_DEBUG_STREAM(get_logger(), "Reconnected.");
      }
      if (!operation(m_cntx)) {
        if (m_isConnected && !isNewConnect) {
          RCLCPP_DEBUG_STREAM(get_logger(),
            "Modbus operation failed. Modbus error: " << modbus_strerror(errno));
          RCLCPP_DEBUG_STREAM(get_logger(), "Trying to re-establish link...");
          modbus_close(m_cntx);
          m_isConnected = false;
        }
        throw Exception::createError(
            "Modbus operation failed after link was re-established. Modbus error: "
          , modbus_strerror(errno));
      }
      break;
    } catch (const std::exception& e) {
      if (attempts) {
        RCLCPP_WARN_STREAM(get_logger(), e.what());
        continue;
      }
      throw;
    }
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Modbus operation successful.");
} defaultCatch

bool Controller::readCoil(const COIL address) try {
  uint8_t result{};
  runModbusOperation([&result, &address](modbus_t* cntx){
    constexpr int cnt = 1;
    return cnt == modbus_read_bits(cntx, (int) address, cnt, &result);
  });
  return 0 != result;
} defaultCatch

void Controller::writeCoil(const COIL address, const bool value) try {
  const int status = value ? 1 : 0;
  runModbusOperation([&address, &status](modbus_t* cntx){
    return 1 == modbus_write_bit(cntx, (int) address, status);
  });
} defaultCatch

uint16_t Controller::readInputRegister(const REG_INP address) try {
  return readInputRegisters(address, 1).at(0);
} defaultCatch

std::vector<uint16_t> Controller::readInputRegisters(
  const REG_INP address, const std::size_t cnt
) try {
  std::vector<uint16_t> result(cnt, 0);
  runModbusOperation([&result, &address, &cnt](modbus_t* cntx){
    return (int) cnt == modbus_read_input_registers(
                    cntx, (int) address, cnt, result.data());
  });
  return result;
} defaultCatch

uint16_t Controller::readHoldingRegister(const REG_HLD address) try {
  uint16_t result{};
  runModbusOperation([&result, &address](modbus_t* cntx){
    constexpr int cnt = 1;
    return cnt == modbus_read_registers(cntx, (int) address, cnt, &result);
  });
  return result;
} defaultCatch

void Controller::writeHoldingRegister(
  const REG_HLD address, const uint16_t value
) try {
  runModbusOperation([&address, &value](modbus_t* cntx){
    constexpr int cnt = 1;
    return cnt == modbus_write_registers(cntx, (int) address, cnt, &value);
  });
} defaultCatch

void Controller::brakeSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Brake::Response> res
) try {

} defaultCatch

void Controller::getDataSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::GetData::Request> req
  , std::shared_ptr<domabot_interfaces::srv::GetData::Response> res
) try {

} defaultCatch

void Controller::moveSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::Move::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Move::Response> res
) try {

} defaultCatch

void Controller::setDirectionSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetDirection::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetDirection::Response> res
) try {

} defaultCatch

void Controller::setModeSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetMode::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetMode::Response> res
) try {

} defaultCatch

void Controller::setSettingsSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetSettings::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetSettings::Response> res
) try {

} defaultCatch

void Controller::stopSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res
) try {

} defaultCatch


void Controller::statusTimerCallback() try {
  const auto inputRegs = readInputRegisters(
    REG_INP::STS, (size_t)REG_INP::END - (size_t)REG_INP::STS);

  auto msg = domabot_interfaces::msg::Status();
  msg.controller.status      = inputRegs[0];
  msg.stepper_left.status    = inputRegs[1];
  msg.stepper_left.position  = inputRegs[2];
  msg.stepper_right.status   = inputRegs[3];
  msg.stepper_right.position = inputRegs[4];

  m_pubStatus->publish(msg);
} catch (const std::exception& e) {
  RCLCPP_INFO_STREAM(get_logger(), e.what());
}

} // Domabot