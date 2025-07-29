
#include <domabot_controller/Controller.h>
#include <domabot_controller/ControllerParams.h>

namespace Domabot {

Controller::Controller() try : Node("domabot_controller") {

  m_modbus = std::make_shared<Modbus>(
    get_logger()
    , ControllerParams::getPath    (*this).c_str()
    , ControllerParams::getBaudRate(*this)
    , ControllerParams::getParity  (*this)
    , ControllerParams::getDataBits(*this)
    , ControllerParams::getStopBits(*this)
    , ControllerParams::getSlaveId (*this)
  );

  const auto protocolVersion = m_modbus->readInputRegister(REG_INP::VER);
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

  const auto pubRate = m_modbus->readHoldingRegister(REG_HLD::RATE);
  const std::chrono::milliseconds pubPeriod(1000 / pubRate);

  m_statusTimer = create_wall_timer(
      pubPeriod, std::bind(&Controller::statusTimerCallback, this));
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

void Controller::checkDirection(const DIR direction) try {
  switch (direction) {
    case DIR::BACKWARD: { [[fallthrough]]; }
    case DIR::FORWARD:  { [[fallthrough]]; }
    case DIR::LEFT:     { [[fallthrough]]; }
    case DIR::RIGHT:    { [[fallthrough]]; }
    case DIR::STOP:     { break; }
    default: { throw Exception::createError("Unknown direction!"); }
  }
} defaultCatch

void Controller::runCommand(const CMD cmd) try {
  RCLCPP_INFO_STREAM(get_logger(), "Start running command: " << getCommandName(cmd));

  m_modbus->writeHoldingRegister(REG_HLD::CMD, (uint16_t) cmd);
  m_modbus->writeCoils(COIL::START, {true, false});

  RCLCPP_DEBUG_STREAM(get_logger(), "Command sended");

  rclcpp::Rate loopRate(2);
  bool isCompleted = false;
  bool isAccepted = !m_modbus->readCoil(COIL::NEW_CMD);
  size_t counter = 0;
  while (rclcpp::ok() && !isCompleted) {
    loopRate.sleep();

    if (!isAccepted) {
      isAccepted = !m_modbus->readCoil(COIL::NEW_CMD);
      RCLCPP_DEBUG_STREAM(get_logger(), "Waiting accept... " << ++counter);
      continue;
    }

    isCompleted = m_modbus->readCoil(COIL::NEW_STS);
    RCLCPP_DEBUG_STREAM(get_logger(), "Waiting complete... " << ++counter);
  }

  if (!isCompleted) {
    throw Exception::createError("Invoked rclcpp::shutdown!");
  }

  const STS status = (STS) m_modbus->readInputRegister(REG_INP::STS);
  m_modbus->writeCoil(COIL::NEW_STS, false);
  checkStatus(status);
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
    [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::GetData::Request> req
  , std::shared_ptr<domabot_interfaces::srv::GetData::Response> res
) try {
  const Modbus::InputRegisters inputRegs = m_modbus->readInputRegisters(
    REG_INP::STS, (uint8_t) REG_INP::END - (uint8_t) REG_INP::STS);
  const Modbus::HoldingRegisters holdingRegs = m_modbus->readHoldingRegisters(
    REG_HLD::START, (size_t) REG_HLD::END);

  auto& status = res->status;
  status.controller.status      = inputRegs.at(REG_INP::STS);
  status.stepper_left.status    = inputRegs.at(REG_INP::STPR_L);
  status.stepper_left.position  = inputRegs.at(REG_INP::POS_L);
  status.stepper_right.status   = inputRegs.at(REG_INP::STPR_R);
  status.stepper_right.position = inputRegs.at(REG_INP::POS_R);

  res->mode.mode           = holdingRegs.at(REG_HLD::MODE);
  res->command.command     = holdingRegs.at(REG_HLD::CMD);
  res->direction.direction = holdingRegs.at(REG_HLD::DIR);

  auto& settings = res->settings;
  res->settings.update_rate = holdingRegs.at(REG_HLD::RATE);

  auto& stepper_left = settings.stepper_left;
  stepper_left.target            = holdingRegs.at(REG_HLD::TARG_L);
  stepper_left.max_speed         = holdingRegs.at(REG_HLD::MAX_SPD_L);
  stepper_left.max_acceleration  = holdingRegs.at(REG_HLD::MAX_ACC_L);
  stepper_left.gear_ratio        = holdingRegs.at(REG_HLD::GEAR_L);
  stepper_left.wheel_diameter    = holdingRegs.at(REG_HLD::WHEEL_DIAM_L);
  stepper_left.is_forward        = holdingRegs.at(REG_HLD::IS_FROWARD_L);

  auto& stepper_right = settings.stepper_right;
  stepper_right.target           = holdingRegs.at(REG_HLD::TARG_R);
  stepper_right.max_speed        = holdingRegs.at(REG_HLD::MAX_SPD_R);
  stepper_right.max_acceleration = holdingRegs.at(REG_HLD::MAX_ACC_R);
  stepper_right.gear_ratio       = holdingRegs.at(REG_HLD::GEAR_R);
  stepper_right.wheel_diameter   = holdingRegs.at(REG_HLD::WHEEL_DIAM_R);
  stepper_right.is_forward       = holdingRegs.at(REG_HLD::IS_FROWARD_R);

  res->response_data.is_success = true;
  res->response_data.error_message = "";

} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::GetData>(res, e);
}

void Controller::moveSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::Move::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Move::Response> res
) try {
  m_modbus->writeHoldingRegisters(
      REG_HLD::TARG_L
    , { (uint16_t) req->target_position_left, (uint16_t) req->target_position_right });

  processRequestCommand<domabot_interfaces::srv::Move>(res, CMD::MOVE);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::Move>(res, e);
}

void Controller::setDirectionSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetDirection::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetDirection::Response> res
) try {
  checkDirection((DIR) req->direction.direction);
  m_modbus->writeHoldingRegister(REG_HLD::DIR, req->direction.direction);

  processRequestCommand<domabot_interfaces::srv::SetDirection>(res, CMD::MODE);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::SetDirection>(res, e);
}

void Controller::setModeSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetMode::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetMode::Response> res
) try {
  checkMode((MODE) req->mode.mode);
  m_modbus->writeHoldingRegister(REG_HLD::MODE, req->mode.mode);

  processRequestCommand<domabot_interfaces::srv::SetMode>(res, CMD::MODE);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::SetMode>(res, e);
}

void Controller::setSettingsSrvCallback(
    const std::shared_ptr<domabot_interfaces::srv::SetSettings::Request> req
  , std::shared_ptr<domabot_interfaces::srv::SetSettings::Response> res
) try {
  Modbus::HoldingRegisters holdingRegs;

  const auto& settings = req->settings;
  holdingRegs.emplace(REG_HLD::RATE, settings.update_rate);

  const auto& stepper_left = settings.stepper_left;
  holdingRegs.emplace(REG_HLD::TARG_L,       stepper_left.target);
  holdingRegs.emplace(REG_HLD::MAX_SPD_L,    stepper_left.max_speed);
  holdingRegs.emplace(REG_HLD::MAX_ACC_L,    stepper_left.max_acceleration);
  holdingRegs.emplace(REG_HLD::GEAR_L,       stepper_left.gear_ratio);
  holdingRegs.emplace(REG_HLD::WHEEL_DIAM_L, stepper_left.wheel_diameter);
  holdingRegs.emplace(REG_HLD::IS_FROWARD_L, stepper_left.is_forward);

  const auto& stepper_right = settings.stepper_right;
  holdingRegs.emplace(REG_HLD::TARG_R,       stepper_right.target);
  holdingRegs.emplace(REG_HLD::MAX_SPD_R,    stepper_right.max_speed);
  holdingRegs.emplace(REG_HLD::MAX_ACC_R,    stepper_right.max_acceleration);
  holdingRegs.emplace(REG_HLD::GEAR_R,       stepper_right.gear_ratio);
  holdingRegs.emplace(REG_HLD::WHEEL_DIAM_R, stepper_right.wheel_diameter);
  holdingRegs.emplace(REG_HLD::IS_FROWARD_R, stepper_right.is_forward);

  //m_modbus->writeHoldingRegisters(holdingRegs); // TODO
  processRequestCommand<domabot_interfaces::srv::SetSettings>(res, CMD::UPDATE);
}  catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::SetSettings>(res, e);
}

void Controller::stopSrvCallback(
    [[maybe_unused]] const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
  , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res
) try {
  processRequestCommand<domabot_interfaces::srv::Stop>(res, CMD::STOP);
} catch (const std::exception& e) {
  processExceptionCommand<domabot_interfaces::srv::Stop>(res, e);
}

void Controller::statusTimerCallback() try {
  const auto inputRegs = m_modbus->readInputRegisters(
    REG_INP::STS, (size_t)REG_INP::END - (size_t)REG_INP::STS);

  auto msg = domabot_interfaces::msg::Status();
  msg.controller.status      = inputRegs.at(REG_INP::STS);
  msg.stepper_left.status    = inputRegs.at(REG_INP::STPR_L);
  msg.stepper_left.position  = inputRegs.at(REG_INP::POS_L);
  msg.stepper_right.status   = inputRegs.at(REG_INP::STPR_R);
  msg.stepper_right.position = inputRegs.at(REG_INP::POS_R);

  m_pubStatus->publish(msg);
} catch (const std::exception& e) {
  RCLCPP_INFO_STREAM(get_logger(), e.what());
}

} // Domabot