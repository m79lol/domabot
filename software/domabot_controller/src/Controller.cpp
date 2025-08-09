/**
 * @file Controller.cpp
 * @brief Domabot Controller class source file.
 * @copyright Copyright 2025 m79lol
*/

#include <domabot_controller/Controller.h>
#include <domabot_controller/ControllerParams.h>

namespace Domabot {
#define DI domabot_interfaces
const char Controller::m_statusTopicName[] = "status";

Controller::Controller() try : Node("domabot_controller") {
  m_modbus = std::make_shared<Modbus>(
      get_logger().get_child("Modbus")
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

  const Modbus::HoldingRegistersValues holdingRegs = m_modbus->readHoldingRegisters({
    REG_HLD::RATE,
    REG_HLD::MODE
  });

  const auto pubRate = holdingRegs.at(REG_HLD::RATE);
  m_currentMode = (MODE) holdingRegs.at(REG_HLD::MODE);
  checkMode(m_currentMode, true);

  m_pubStatus = create_publisher<DI::msg::Status>(
    m_statusTopicName, 1);

  m_timerCallbackGroup =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const rclcpp::CallbackGroup::SharedPtr serviceCallbackGroup =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  m_srvBrake = create_service<DI::srv::Brake>(
      "brake"
    , std::bind(
        &Controller::brakeSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvGetData = create_service<DI::srv::GetData>(
      "get_data"
    , std::bind(
        &Controller::getDataSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvMove = create_service<DI::srv::Move>(
      "move"
    , std::bind(
        &Controller::moveSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvSaveSettings = create_service<DI::srv::SaveSettings>(
      "save_settings"
    , std::bind(
        &Controller::saveSettingsSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvSetDirection = create_service<DI::srv::SetDirection>(
      "set_direction"
    , std::bind(
        &Controller::setDirectionSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvSetMode = create_service<DI::srv::SetMode>(
      "set_mode"
    , std::bind(
        &Controller::setModeSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvSetSettings = create_service<DI::srv::SetSettings>(
      "set_settings"
    , std::bind(
        &Controller::setSettingsSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );
  m_srvStop = create_service<DI::srv::Stop>(
      "stop"
    , std::bind(
        &Controller::stopSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , serviceCallbackGroup
  );

  restartStatusTimer(pubRate);

  m_statsTimer = create_wall_timer(
      std::chrono::milliseconds(1000)
    , std::bind(&Controller::statsTimerCallback, this)
    , m_timerCallbackGroup
  );

  RCLCPP_INFO_STREAM(
      get_logger()
    , "Node " << get_name() << " inited, with protocol version: "
      << protocolVersion);

  switch (m_currentMode) {
    case MODE::TRG: {
      runCommand(CMD::BRAKE);
      break;
    }
    case MODE::DRCT: {
      m_modbus->writeHoldingRegister(REG_HLD::DIR, (uint16_t) DIR::STOP);
      runCommand(CMD::DIR);
      break;
    }
    case MODE::WRD: { break; }
    default:        { break; }
  }
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
      throw Exception::createError("Unknow command ", (uint8_t) command, "!");
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
    CASE_STATUS(EMERGENCY);
    CASE_STATUS(ERR_UNKNOWN);
    default: {
      throw Exception::createError("Unknow status ", (uint8_t) status, "!");
    }
  }
  #undef CASE_STATUS
} defaultCatch

const std::string& Controller::getModeName(const MODE mode) try {
  #define CASE_MODE(MODE_) CASE_ITEM(MODE, MODE_)
  switch (mode) {
    CASE_MODE(TRG);
    CASE_MODE(DRCT);
    CASE_MODE(WRD);
    default: {
      throw Exception::createError("Unknow mode ", (uint8_t) mode, "!");
    }
  }
  #undef CASE_MODE
} defaultCatch

#undef CASE_ITEM

void Controller::checkStatus(const STS status, const bool selfCheck) const try {
  switch (status) {
    case STS::OK:          { break; }
    case STS::ERR_MOVING:  { break; }
    case STS::ERR_CMD:     { break; }
    case STS::ERR_PARAMS:  { break; }
    case STS::ERR_MODE:    { break; }
    case STS::ERR_DIR:     { break; }
    case STS::EMERGENCY:   { break; }
    case STS::ERR_UNKNOWN: { break; }
    default: {
      if (selfCheck) {
        throw Exception::createError("Unknow status ", (uint8_t) status, "!");
      }
    }
  }

  if (selfCheck) {
    return;
  }

  const std::string msg = Exception::createMsg(
    "Command execution status: ", getStatusName(status));
  if (STS::OK == status) {
    RCLCPP_INFO_STREAM(get_logger(), msg);
    return;
  }

  RCLCPP_ERROR_STREAM(get_logger(), msg);
  throw Exception::createError(msg);
} defaultCatch

void Controller::checkMode(const MODE mode, const bool selfCheck) try {
  switch (mode) {
    case MODE::TRG:  { break; }
    case MODE::DRCT: { break; }
    case MODE::WRD: {
      if (!selfCheck) {
        throw Exception::createError(
          "Mode wired may activate only by hardware switch!");
      }
      break;
    }
    default: { throw Exception::createError("Unknown mode: ", (uint8_t) mode); }
  }
} defaultCatch

void Controller::checkDirection(const DIR direction) try {
  switch (direction) {
    case DIR::BACKWARD: { break; }
    case DIR::FORWARD:  { break; }
    case DIR::LEFT:     { break; }
    case DIR::RIGHT:    { break; }
    case DIR::STOP:     { break; }
    default: {
      throw Exception::createError("Unknown direction: ", (uint8_t) direction); }
  }
} defaultCatch

void Controller::checkStepperStatus(const STPR_STS stepperStatus) try {
  switch (stepperStatus) {
    case STPR_STS::STOPPED:               { break; }
    case STPR_STS::MOVING_TO_TARGET:      { break; }
    case STPR_STS::MOVING_TO_PAUSE_POINT: { break; }
    case STPR_STS::MOVING_AT_SPEED:       { break; }
    case STPR_STS::SLOWING_DOWN:          { break; }
    default: {
      throw Exception::createError(
        "Unknown stepper status: ", (uint8_t) stepperStatus);
    }
  }
} defaultCatch

void Controller::checkCommand(const CMD command) try {
  switch (command) {
    case CMD::BRAKE:  { break; }
    case CMD::STOP:   { break; }
    case CMD::MOVE:   { break; }
    case CMD::UPDATE: { break; }
    case CMD::SAVE:   { break; }
    case CMD::MODE:   { break; }
    case CMD::DIR:    { break; }
    default: {
      throw Exception::createError("Unknown command: ", (uint8_t) command); }
  }
} defaultCatch

void Controller::setSettingsToRegisters(
      const DI::msg::ControllerSettings& settings) try {
  Modbus::HoldingRegistersValues holdingRegs;

  setRegister(holdingRegs, REG_HLD::RATE, settings.update_rate, "update_rate", true);

  if (!settings.stepper_left.empty()) {
    const auto& stepper_left = settings.stepper_left.front();
    setRegister(holdingRegs, REG_HLD::TARG_L, stepper_left.target);
    setRegister(holdingRegs, REG_HLD::MAX_SPD_L, stepper_left.max_speed,
      "stepper_left.max_speed", true);
    setRegister(holdingRegs, REG_HLD::MAX_ACC_L, stepper_left.max_acceleration,
      "stepper_left.max_acceleration", true);
    setRegister(holdingRegs, REG_HLD::GEAR_L, stepper_left.gear_ratio,
      "stepper_left.gear_ratio", true);
    setRegister(holdingRegs, REG_HLD::WHEEL_DIAM_L, stepper_left.wheel_diameter,
      "stepper_left.wheel_diameter", true);
    setRegister(holdingRegs, REG_HLD::IS_FROWARD_L, stepper_left.is_forward);
  }

  if (!settings.stepper_right.empty()) {
    const auto& stepper_right = settings.stepper_right.front();
    setRegister(holdingRegs, REG_HLD::TARG_R, stepper_right.target);
    setRegister(holdingRegs, REG_HLD::MAX_SPD_R, stepper_right.max_speed,
      "stepper_right.max_speed", true);
    setRegister(holdingRegs, REG_HLD::MAX_ACC_R, stepper_right.max_acceleration,
      "stepper_right.max_acceleration", true);
    setRegister(holdingRegs, REG_HLD::GEAR_R, stepper_right.gear_ratio,
      "stepper_right.gear_ratio", true);
    setRegister(holdingRegs, REG_HLD::WHEEL_DIAM_R, stepper_right.wheel_diameter,
      "stepper_right.wheel_diameter", true);
    setRegister(holdingRegs, REG_HLD::IS_FROWARD_R, stepper_right.is_forward);
  }

  m_modbus->writeHoldingRegisters(holdingRegs);
} defaultCatch

void Controller::runCommand(const CMD cmd) try {
  RCLCPP_INFO_STREAM(get_logger(), "Execute command: " << getCommandName(cmd));

  checkCommand(cmd);

  switch (cmd) {
    case CMD::MODE: { [[fallthrough]]; }
    case CMD::MOVE: { [[fallthrough]]; }
    case CMD::SAVE: { [[fallthrough]]; }
    case CMD::UPDATE: {
      if (m_isCommandExecuting) {
        throw Exception::createError(
            "Can't execute command ", getCommandName(cmd)
          , " during executing another command!");
      }
      break;
    }
    case CMD::DIR: {
      if (MODE::DRCT != m_currentMode) {
        throw Exception::createError(
            "Can't execute command ", getCommandName(cmd)
          , " in mode ", getModeName(m_currentMode)
          , "! This command execute only in ", getModeName(MODE::DRCT), " mode.");
      }
      break;
    }

    // can execute at any time
    case CMD::BRAKE: { break; }
    case CMD::STOP:  { break; }
  }

  m_modbus->writeHoldingRegister(REG_HLD::CMD, (uint16_t) cmd);
  m_modbus->writeCoils({{COIL::NEW_CMD, true}, {COIL::NEW_STS, false}});

  RCLCPP_DEBUG_STREAM(get_logger(), "Command sended");

  rclcpp::Rate loopRate(2);
  bool isCompleted = false;
  bool isAccepted = !m_modbus->readCoil(COIL::NEW_CMD);
  size_t counter = 0;
  while (rclcpp::ok() && !isCompleted) {
    loopRate.sleep();

    if (!isAccepted) {
      if (counter >= 5) {
        throw Exception::createError(
          "Microcontroller does not accept command within ", counter, " attempts!");
      }
      isAccepted = !m_modbus->readCoil(COIL::NEW_CMD);
      RCLCPP_DEBUG_STREAM(get_logger(), "Waiting accept... " << ++counter);
      m_isCommandExecuting = true;
      if (!isAccepted) {
        continue;
      }
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

void Controller::restartStatusTimer(const uint16_t rate) try {
  if (0 == rate) {
    throw Exception::createError("Invalid update status rate!");
  }

  const size_t cntScrbrs = m_cntStatusSubscriber.load(std::memory_order_acquire);
  {
    const std::lock_guard<std::mutex> lock(m_mtxTimer);
    if (!m_statusTimer->is_canceled() && rate == m_statusRate && 0 < cntScrbrs) {
      return;
    }
    if (nullptr != m_statusTimer) {
      m_statusTimer->cancel();  // stop old timer
    }
    if (0 == cntScrbrs) {
      return;
    }

    const std::chrono::milliseconds pubPeriod(1000 / rate);
    m_statusTimer = create_wall_timer(
        pubPeriod
      , std::bind(&Controller::statusTimerCallback, this)
      , m_timerCallbackGroup
    );
    m_statusRate = rate;
  }

  RCLCPP_INFO_STREAM(
      get_logger()
    , "Restarted status timer with new rare: " << rate << " hz");
} defaultCatch

void Controller::brakeSrvCallback(
    [[maybe_unused]] const std::shared_ptr<DI::srv::Brake::Request> req
  , std::shared_ptr<DI::srv::Brake::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "Brake service called.");
  processRequestCommand<DI::srv::Brake>(res, CMD::BRAKE);
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::Brake>(res, e);
}

void Controller::getDataSrvCallback(
    [[maybe_unused]] const std::shared_ptr<DI::srv::GetData::Request> req
  , std::shared_ptr<DI::srv::GetData::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "GetData service called.");

  const Modbus::InputRegistersValues inputRegs = m_modbus->readInputRegisters({
      REG_INP::STS
    , REG_INP::STPR_L
    , REG_INP::POS_L
    , REG_INP::STPR_R
    , REG_INP::POS_R
    , REG_INP::END
  });
  const Modbus::HoldingRegistersValues holdingRegs = m_modbus->readHoldingRegisters({
      REG_HLD::CMD
    , REG_HLD::TARG_L
    , REG_HLD::TARG_R
    , REG_HLD::RATE
    , REG_HLD::MAX_SPD_L
    , REG_HLD::MAX_ACC_L
    , REG_HLD::GEAR_L
    , REG_HLD::WHEEL_DIAM_L
    , REG_HLD::IS_FROWARD_L
    , REG_HLD::MAX_SPD_R
    , REG_HLD::MAX_ACC_R
    , REG_HLD::GEAR_R
    , REG_HLD::WHEEL_DIAM_R
    , REG_HLD::IS_FROWARD_R
    , REG_HLD::MODE
    , REG_HLD::DIR
  });

  checkStatus((STS) inputRegs.at(REG_INP::STS), true);

  checkCommand((CMD) holdingRegs.at(REG_HLD::CMD));
  checkMode((MODE) holdingRegs.at(REG_HLD::MODE), true);
  checkDirection((DIR) holdingRegs.at(REG_HLD::DIR));

  checkStepperStatus((STPR_STS) inputRegs.at(REG_INP::STPR_L));
  checkStepperStatus((STPR_STS) inputRegs.at(REG_INP::STPR_R));

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
  res->settings.update_rate.push_back(holdingRegs.at(REG_HLD::RATE));
  restartStatusTimer(holdingRegs.at(REG_HLD::RATE));

  DI::msg::StepperSettings stepper_left;
  stepper_left.target          .push_back(holdingRegs.at(REG_HLD::TARG_L));
  stepper_left.max_speed       .push_back(holdingRegs.at(REG_HLD::MAX_SPD_L));
  stepper_left.max_acceleration.push_back(holdingRegs.at(REG_HLD::MAX_ACC_L));
  stepper_left.gear_ratio      .push_back(holdingRegs.at(REG_HLD::GEAR_L));
  stepper_left.wheel_diameter  .push_back(holdingRegs.at(REG_HLD::WHEEL_DIAM_L));
  stepper_left.is_forward      .push_back(holdingRegs.at(REG_HLD::IS_FROWARD_L));
  settings.stepper_left.push_back(stepper_left);

  DI::msg::StepperSettings stepper_right;
  stepper_right.target          .push_back(holdingRegs.at(REG_HLD::TARG_R));
  stepper_right.max_speed       .push_back(holdingRegs.at(REG_HLD::MAX_SPD_R));
  stepper_right.max_acceleration.push_back(holdingRegs.at(REG_HLD::MAX_ACC_R));
  stepper_right.gear_ratio      .push_back(holdingRegs.at(REG_HLD::GEAR_R));
  stepper_right.wheel_diameter  .push_back(holdingRegs.at(REG_HLD::WHEEL_DIAM_R));
  stepper_right.is_forward      .push_back(holdingRegs.at(REG_HLD::IS_FROWARD_R));
  settings.stepper_left.push_back(stepper_right);

  res->response_data.is_success = true;
  res->response_data.error_message = "";
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::GetData>(res, e);
}

void Controller::moveSrvCallback(
    const std::shared_ptr<DI::srv::Move::Request> req
  , std::shared_ptr<DI::srv::Move::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "Move service called.");

  m_modbus->writeHoldingRegisters({
      { REG_HLD::TARG_L, (uint16_t) req->target_position_left }
    , { REG_HLD::TARG_R, (uint16_t) req->target_position_right }
  });

  processRequestCommand<DI::srv::Move>(res, CMD::MOVE);
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::Move>(res, e);
}

void Controller::saveSettingsSrvCallback(
    const std::shared_ptr<DI::srv::SaveSettings::Request> req
  , std::shared_ptr<DI::srv::SaveSettings::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "SaveSettings service called.");

  const auto& settings = req->settings;
  if (!settings.empty()) {
    setSettingsToRegisters(settings.front());
  }

  processRequestCommand<DI::srv::SaveSettings>(res, CMD::SAVE);

  if (!settings.empty()) {
    if (!settings.front().update_rate.empty()) {
      restartStatusTimer(settings.front().update_rate.front());
    }
  }
}  catch (const std::exception& e) {
  processExceptionCommand<DI::srv::SaveSettings>(res, e);
}

void Controller::setDirectionSrvCallback(
    const std::shared_ptr<DI::srv::SetDirection::Request> req
  , std::shared_ptr<DI::srv::SetDirection::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "SetDirection service called.");

  checkDirection((DIR) req->direction.direction);
  m_modbus->writeHoldingRegister(REG_HLD::DIR, req->direction.direction);

  processRequestCommand<DI::srv::SetDirection>(res, CMD::DIR);
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::SetDirection>(res, e);
}

void Controller::setModeSrvCallback(
    const std::shared_ptr<DI::srv::SetMode::Request> req
  , std::shared_ptr<DI::srv::SetMode::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "SetMode service called.");

  checkMode((MODE) req->mode.mode);
  m_modbus->writeHoldingRegister(REG_HLD::MODE, req->mode.mode);

  processRequestCommand<DI::srv::SetMode>(res, CMD::MODE);
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::SetMode>(res, e);
}

void Controller::setSettingsSrvCallback(
    const std::shared_ptr<DI::srv::SetSettings::Request> req
  , std::shared_ptr<DI::srv::SetSettings::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "SetSettings service called.");

  const auto& settings = req->settings;
  setSettingsToRegisters(settings);

  processRequestCommand<DI::srv::SetSettings>(res, CMD::UPDATE);

  if (!settings.update_rate.empty()) {
    restartStatusTimer(settings.update_rate.front());
  }
}  catch (const std::exception& e) {
  processExceptionCommand<DI::srv::SetSettings>(res, e);
}

void Controller::stopSrvCallback(
    [[maybe_unused]] const std::shared_ptr<DI::srv::Stop::Request> req
  , std::shared_ptr<DI::srv::Stop::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "Stop service called.");
  processRequestCommand<DI::srv::Stop>(res, CMD::STOP);
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::Stop>(res, e);
}

void Controller::statusTimerCallback() try {
  const auto inputRegs = m_modbus->readInputRegisters({
      REG_INP::STS
    , REG_INP::STPR_L
    , REG_INP::POS_L
    , REG_INP::STPR_R
    , REG_INP::POS_R
  });

  auto msg = DI::msg::Status();
  msg.controller.status      = inputRegs.at(REG_INP::STS);
  msg.stepper_left.status    = inputRegs.at(REG_INP::STPR_L);
  msg.stepper_left.position  = inputRegs.at(REG_INP::POS_L);
  msg.stepper_right.status   = inputRegs.at(REG_INP::STPR_R);
  msg.stepper_right.position = inputRegs.at(REG_INP::POS_R);

  m_pubStatus->publish(msg);
} catch (const std::exception& e) {
  RCLCPP_ERROR_STREAM(get_logger(), e.what());
}

void Controller::statsTimerCallback() try {
  m_cntStatusSubscriber.store(
    count_subscribers(m_statusTopicName), std::memory_order_relaxed);
  restartStatusTimer(m_statusRate);
} catch (const std::exception& e) {
  RCLCPP_ERROR_STREAM(get_logger(), e.what());
}

#undef DI
}  // namespace Domabot
