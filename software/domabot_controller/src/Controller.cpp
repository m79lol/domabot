
#include <domabot_controller/Controller.h>
#include <domabot_controller/ControllerParams.h>


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
  return writeCoils(address, {value});
} defaultCatch

void Controller::writeCoils(
  const COIL startAddress, const std::vector<bool>& values
) try {
  std::vector<uint8_t> coils(values.size(), 0);
  for (size_t i = 0; i < values.size(); ++i) {
    coils[i] = values[i] ? 1 : 0;
  }
  runModbusOperation([&startAddress, &coils](modbus_t* cntx){
    return (int) coils.size() == modbus_write_bits(cntx, (int) startAddress, (int) coils.size(), coils.data());
  });
} defaultCatch

uint16_t Controller::readInputRegister(const REG_INP address) try {
  return readInputRegisters(address, 1).at(0);
} defaultCatch

std::vector<uint16_t> Controller::readInputRegisters(
  const REG_INP startAddress, const std::size_t cnt
) try {
  std::vector<uint16_t> result(cnt, 0);
  runModbusOperation([&result, &startAddress, &cnt](modbus_t* cntx){
    return (int) cnt == modbus_read_input_registers(
                    cntx, (int) startAddress, cnt, result.data());
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

#define CASE_ITEM(TYPE, ITEM) \
  case TYPE::ITEM: { \
    static const std::string str = #ITEM; \
    return str; \
  }

const std::string& Controller::getCommandName(const CMD command) try {
  #define CASE_COMMAND(COMMAND) CASE_ITEM(CMD, COMMAND)
  switch (command) {
    CASE_COMMAND(BRAKE);
    CASE_COMMAND(DIR);
    CASE_COMMAND(MODE);
    CASE_COMMAND(MOVE);
    CASE_COMMAND(STOP);
    CASE_COMMAND(UPDATE);
    default: {
      throw Exception::createError("Unknow command type!");
    }
  }
  #undef CASE_COMMAND
} defaultCatch

const std::string& Controller::getStatusName(const STS status) try {
  #define CASE_STATUS(STATUS) CASE_ITEM(STS, STATUS)
  switch (status) {
    CASE_STATUS(OK);
    CASE_STATUS(ERR_MOVING);
    CASE_STATUS(ERR_CMD);
    CASE_STATUS(ERR_PARAMS);
    CASE_STATUS(ERR_MODE);
    CASE_STATUS(ERR_DIR);
    CASE_STATUS(EMRGENCY);
    CASE_STATUS(ERR_UNKNOWN);
    default: {
      throw Exception::createError("Unknow status type!");
    }
  }
  #undef CASE_STATUS
} defaultCatch

#undef CASE_ITEM

void Controller::checkStatus(const STS status) const try {
  const std::string msg = Exception::createMsg("Command execution status: ", getStatusName(status));
  if (STS::OK == status) {
    RCLCPP_INFO_STREAM(get_logger(), msg);
    return;
  }

  RCLCPP_ERROR_STREAM(get_logger(), msg);
  throw Exception::createError(msg);
} defaultCatch

void Controller::checkMode(const MODE mode) try {
  switch (mode) {
    case MODE::TRG:  { [[fallthrough]]; }
    case MODE::DRCT: { break; }
    case MODE::WRD: {
      throw Exception::createError("Mode wired activated only by hardware switch!");
    }
    default: { throw Exception::createError("Unknown mode!"); }
  }
} defaultCatch

STS Controller::runCommand(const CMD cmd) try {
  RCLCPP_INFO_STREAM(get_logger(), "Start running command: " << getCommandName(cmd));

  writeHoldingRegister(REG_HLD::CMD, (uint16_t) cmd);
  writeCoils(COIL::START, {true, false});

  RCLCPP_DEBUG_STREAM(get_logger(), "Command sended");

  rclcpp::Rate loopRate(2);
  bool isCompleted = false;
  bool isAccepted = !readCoil(COIL::NEW_CMD);
  size_t counter = 0;
  while (rclcpp::ok() && !isCompleted) {
    loopRate.sleep();

    if (!isAccepted) {
      isAccepted = !readCoil(COIL::NEW_CMD);
      RCLCPP_DEBUG_STREAM(get_logger(), "Waiting accept... " << ++counter);
      continue;
    }

    isCompleted = readCoil(COIL::NEW_STS);
    RCLCPP_DEBUG_STREAM(get_logger(), "Waiting complete... " << ++counter);
  }

  if (!isCompleted) {
    throw Exception::createError("Invoked rclcpp::shutdown!");
  }

  const STS status = (STS)readInputRegister(REG_INP::STS);
  writeCoil(COIL::NEW_STS, false);

  return status;
} defaultCatch

void Controller::brakeSrvCallback(
    [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Brake::Response> res
) try {
  processRequestCommand<domabot_interfaces::srv::Brake>(res, CMD::BRAKE);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::Brake>(res, e);
}

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
  res->controller_status = (uint8_t) STS::ERR_UNKNOWN;
  checkMode((MODE) req->mode.mode);
  writeHoldingRegister(REG_HLD::MODE, req->mode.mode);

  processRequestCommand<domabot_interfaces::srv::SetMode>(res, CMD::MODE);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::SetMode>(res, e);
}

void Controller::setSettingsSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetSettings::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetSettings::Response> res
) try {

} defaultCatch

void Controller::stopSrvCallback(
    [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res
) try {
  processRequestCommand<domabot_interfaces::srv::Stop>(res, CMD::STOP);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::Stop>(res, e);
}

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