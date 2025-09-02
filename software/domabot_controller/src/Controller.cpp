/**
 * @file Controller.cpp
 * @brief Domabot Controller class source file.
 * @copyright Copyright 2025 m79lol
*/

#include <domabot_controller/Controller.h>
#include <domabot_controller/ControllerParams.h>

namespace Domabot {
#define DI domabot_controller_interfaces
const char Controller::m_statusTopicName[] = "status";

Controller::Controller() try : Node("domabot_controller") {
  m_modbus = std::make_shared<Modbus>(
      get_logger().get_child("Modbus")
    , ControllerParams::getPath         (*this).c_str()
    , ControllerParams::getBaudRate     (*this)
    , ControllerParams::getParity       (*this).at(0)
    , ControllerParams::getDataBits     (*this)
    , ControllerParams::getStopBits     (*this)
    , ControllerParams::getSlaveId      (*this)
    , ControllerParams::getConnectDelay (*this)
    , ControllerParams::getModbusTimeout(*this)
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
  m_currentMode = checkMode(holdingRegs.at(REG_HLD::MODE), true);
  m_isMotorsEnabled = m_modbus->readCoil(COIL::ENBL);

  m_pubStatus = create_publisher<DI::msg::Status>(
    m_statusTopicName, 1);

  m_timerCallbackGroup =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  m_serviceCallbackGroup =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  m_srvBrake = create_service<DI::srv::Brake>(
      "brake"
    , std::bind(
        &Controller::brakeSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvEnableMotors = create_service<DI::srv::EnableMotors>(
      "enable_motors"
    , std::bind(
        &Controller::enableMotorsSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvGetData = create_service<DI::srv::GetData>(
      "get_data"
    , std::bind(
        &Controller::getDataSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvMove = create_service<DI::srv::Move>(
      "move"
    , std::bind(
        &Controller::moveSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvSaveSettings = create_service<DI::srv::SaveSettings>(
      "save_settings"
    , std::bind(
        &Controller::saveSettingsSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvSetDirection = create_service<DI::srv::SetDirection>(
      "set_direction"
    , std::bind(
        &Controller::setDirectionSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvSetMode = create_service<DI::srv::SetMode>(
      "set_mode"
    , std::bind(
        &Controller::setModeSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvSetSettings = create_service<DI::srv::SetSettings>(
      "set_settings"
    , std::bind(
        &Controller::setSettingsSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
  );
  m_srvStop = create_service<DI::srv::Stop>(
      "stop"
    , std::bind(
        &Controller::stopSrvCallback, this,
        std::placeholders::_1, std::placeholders::_2)
    , rcl_service_get_default_options().qos
    , m_serviceCallbackGroup
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

STS Controller::checkStatus(const uint16_t value, const bool selfCheck) const try {
  const STS status = checkEnumItem<STS>(value, "status", false);
  if (selfCheck) {
    return status;
  }

  const std::string msg = Exception::createMsg(
    "Command execution status: ", magic_enum::enum_name(status));
  if (STS::OK == status) {
    RCLCPP_INFO_STREAM(get_logger(), msg);
    return status;
  }

  RCLCPP_ERROR_STREAM(get_logger(), msg);
  throw Exception::createError(msg);
} defaultCatch

MODE Controller::checkMode(const uint16_t value, const bool selfCheck) try {
  const MODE mode = checkEnumItem<MODE>(value, "mode", false);
  if (MODE::WRD == mode && !selfCheck) {
    throw Exception::createError(
      "Mode wired may activate only by hardware switch!");
  }
  return mode;
} defaultCatch

DIR Controller::checkDirection(const uint16_t value) try {
  return checkEnumItem<DIR>(value, "direction");
} defaultCatch

STPR_STS Controller::checkStepperStatus(const uint16_t value) try {
  return checkEnumItem<STPR_STS>(value, "stepper status");
} defaultCatch

CMD Controller::checkCommand(const uint16_t value) try {
  return checkEnumItem<CMD>(value, "command");
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
  checkCommand((uint16_t) cmd);

  const auto commandName = magic_enum::enum_name(cmd);
  RCLCPP_INFO_STREAM(get_logger(), "Execute command: " << commandName);

  switch (cmd) {
    case CMD::BRAKE: { [[fallthrough]]; }
    case CMD::DIR:   { [[fallthrough]]; }
    case CMD::MODE:  { [[fallthrough]]; }
    case CMD::MOVE:  { [[fallthrough]]; }
    case CMD::STOP: {
      if (!m_isMotorsEnabled) {
        throw Exception::createError(
            "Command ", commandName , " requires enabled motors!");
      }
      [[fallthrough]];
    }
    default: { break; }
  }

  switch (cmd) {
    case CMD::ENBL: { [[fallthrough]]; }
    case CMD::MODE: { [[fallthrough]]; }
    case CMD::MOVE: { [[fallthrough]]; }
    case CMD::SAVE: { [[fallthrough]]; }
    case CMD::UPDATE: {
      if (m_isMoving.load(std::memory_order_acquire)) {
        throw Exception::createError(
            "Can't execute command ", commandName
          , " during moving!");
      }
      [[fallthrough]];
    }
    default: { break; }
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

  const uint16_t status = m_modbus->readInputRegister(REG_INP::STS);
  m_modbus->writeCoil(COIL::NEW_STS, false);
  checkStatus(status);
} defaultCatch

void Controller::restartStatusTimer(const uint16_t rate) try {
  if (0 == rate) {
    throw Exception::createError(
      "Invalid update status rate! Rate must be above zero!");
  }

  {
    const std::lock_guard<std::mutex> lock(m_mtxTimer);
    static uint16_t oldRate = 0;
    if (rate == oldRate) {
      return;
    }
    if (nullptr != m_statusTimer && !m_statusTimer->is_canceled()) {
      m_statusTimer->cancel();  // stop old timer
    }

    const std::chrono::milliseconds pubPeriod(1000 / rate);
    m_statusTimer = create_wall_timer(
        pubPeriod
      , std::bind(&Controller::statusTimerCallback, this)
      , m_timerCallbackGroup
    );
    oldRate = rate;
  }

  RCLCPP_INFO_STREAM(
      get_logger()
    , "Restarted status timer with new rate: " << rate << " hz");
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

void Controller::enableMotorsSrvCallback(
    const std::shared_ptr<DI::srv::EnableMotors::Request> req
  , std::shared_ptr<DI::srv::EnableMotors::Response> res
) try {
  RCLCPP_DEBUG_STREAM(get_logger(), "Enable motors service called.");
  m_modbus->writeCoil(COIL::ENBL, req->enable_motors);
  processRequestCommand<DI::srv::EnableMotors>(res, CMD::ENBL);
  m_isMotorsEnabled = req->enable_motors;
} catch (const std::exception& e) {
  processExceptionCommand<DI::srv::EnableMotors>(res, e);
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

  checkStatus(inputRegs.at(REG_INP::STS), true);

  checkCommand(holdingRegs.at(REG_HLD::CMD));
  checkMode(holdingRegs.at(REG_HLD::MODE), true);
  checkDirection(holdingRegs.at(REG_HLD::DIR));

  checkStepperStatus(inputRegs.at(REG_INP::STPR_L));
  checkStepperStatus(inputRegs.at(REG_INP::STPR_R));

  auto& status = res->status;
  status.controller.status      = inputRegs.at(REG_INP::STS);
  status.stepper_left.status    = inputRegs.at(REG_INP::STPR_L);
  status.stepper_left.position  = inputRegs.at(REG_INP::POS_L);
  status.stepper_right.status   = inputRegs.at(REG_INP::STPR_R);
  status.stepper_right.position = inputRegs.at(REG_INP::POS_R);

  res->mode.mode           = holdingRegs.at(REG_HLD::MODE);
  res->command.command     = holdingRegs.at(REG_HLD::CMD);
  res->direction.direction = holdingRegs.at(REG_HLD::DIR);
  res->is_motors_enabled = m_modbus->readCoil(COIL::ENBL);

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
  settings.stepper_right.push_back(stepper_right);

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

  checkAllowedMode(CMD::MOVE, MODE::TRG);

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

  checkDirection(req->direction.direction);
  checkAllowedMode(CMD::DIR, MODE::DRCT);

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

  const MODE tryMode = checkMode(req->mode.mode);
  m_modbus->writeHoldingRegister(REG_HLD::MODE, req->mode.mode);

  processRequestCommand<DI::srv::SetMode>(res, CMD::MODE);
  m_currentMode = tryMode;
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

  const STPR_STS leftStatus = checkStepperStatus(inputRegs.at(REG_INP::STPR_L));
  const STPR_STS rightStatus = checkStepperStatus(inputRegs.at(REG_INP::STPR_R));

  m_isMoving.store(
      leftStatus != STPR_STS::STOPPED
      || rightStatus != STPR_STS::STOPPED
    , std::memory_order_relaxed);

  const bool isScrbrs = m_isStatusSubscriber.load(std::memory_order_acquire);
  if (0 == isScrbrs) {
    return;
  }

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
  const bool oldCnt = m_isStatusSubscriber.load(std::memory_order_acquire);
  const bool newCnt = 0 != count_subscribers(m_statusTopicName);
  if (oldCnt == newCnt) {
    return;
  }
  m_isStatusSubscriber.store(newCnt, std::memory_order_relaxed);
} catch (const std::exception& e) {
  RCLCPP_ERROR_STREAM(get_logger(), e.what());
}

#undef DI
}  // namespace Domabot
