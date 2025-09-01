/**
 * @file CLI.cpp
 * @brief Domabot CLI node class source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_cli/CLI.h>

#include <cstring>
#include <cerrno>

namespace Domabot {

#define DI domabot_controller_interfaces

CLI::CLI() try : Node("domabot_cli") {
  m_subStatus = create_subscription<DI::msg::Status>(
      "status", 1, std::bind(&CLI::statusCallback, this, std::placeholders::_1));

  m_clientBrake        = create_client<DI::srv::Brake>("brake");
  m_clientEnableMotors = create_client<DI::srv::EnableMotors>("enable_motors");
  m_clientGetData      = create_client<DI::srv::GetData>("get_data");
  m_clientMove         = create_client<DI::srv::Move>("move");
  m_clientSaveSettings = create_client<DI::srv::SaveSettings>("save_settings");
  m_clientSetDirection = create_client<DI::srv::SetDirection>("set_direction");
  m_clientSetMode      = create_client<DI::srv::SetMode>("set_mode");
  m_clientSetSettings  = create_client<DI::srv::SetSettings>("set_settings");
  m_clientStop         = create_client<DI::srv::Stop>("stop");
} defaultCatch

bool CLI::inputSettings(DI::msg::ControllerSettings& newSettings) try {
  const auto resData = callService<DI::srv::GetData>(
      m_clientGetData
    , std::make_shared<DI::srv::GetData::Request>()
  );

  const auto& oldSettings = resData->settings;
  bool updated = false;
  updated |= inputSettingValue(
    "update rate", newSettings.update_rate, oldSettings.update_rate);
  updated |= inputStepperSettings(
    newSettings.stepper_left, oldSettings.stepper_left
  );
  updated |= inputStepperSettings(
    newSettings.stepper_right, oldSettings.stepper_right
  );

  return updated;
} defaultCatch

void CLI::statusCallback(const DI::msg::Status& msg) try {
  const std::lock_guard<std::mutex> lock(m_mtxLastStatus);
  m_msgLastStatus = msg;
} catch (const std::exception& e) {
  RCLCPP_ERROR_STREAM(get_logger(), e.what());
}

void CLI::changeDirection(const DIR dir) try {
  const auto req = std::make_shared<DI::srv::SetDirection::Request>();
  req->direction.direction = static_cast<uint8_t>(dir);
  callService<DI::srv::SetDirection>(m_clientSetDirection, req);
} defaultCatch

void CLI::restoreConsoleMode() try {
  tcsetattr(m_kfd, TCSANOW, &m_cooked);
} defaultCatch

void CLI::runDirectMode() try {
  tcgetattr(m_kfd, &m_cooked);
  memcpy(&m_raw, &m_cooked, sizeof(struct termios));
  m_raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  m_raw.c_cc[VEOL] = 1;
  m_raw.c_cc[VEOF] = 2;
  tcsetattr(m_kfd, TCSANOW, &m_raw);

  RCLCPP_INFO_STREAM(
      get_logger()
    , "Reading from keyboard\n"
      << "---------------------------"
      << "Use arrow keys to move the robot.\n"
      << "SPACE key is break. Q key is exit.");

  char key = 0;
  char lastKey = 0;
  bool isQuit = false;

  while (rclcpp::ok()) {
    if (read(m_kfd, &key, 1) < 0) {
      throw Exception::createError("Error read key:", strerror(errno));
    }

    RCLCPP_DEBUG(get_logger(), "key: 0x%02X\n", key);
    if (key == lastKey) {
      continue;
    }

    constexpr char keyR     = 0x43;
    constexpr char keyL     = 0x44;
    constexpr char keyU     = 0x41;
    constexpr char keyD     = 0x42;
    constexpr char keyQ     = 0x71;
    constexpr char keySpace = 0x20;

    DIR dir = DIR::STOP;
    bool isSkip = false;
    switch(key) {
      case keyL:     { dir = DIR::LEFT;     break; }
      case keyU:     { dir = DIR::FORWARD;  break; }
      case keyD:     { dir = DIR::BACKWARD; break; }
      case keyR:     { dir = DIR::RIGHT;    break; }
      case keySpace: { dir = DIR::STOP;     break; }
      case keyQ:     { isQuit = true;       break; }
      default:       { isSkip = true;       break; }
    }
    if (isQuit) { break; }
    if (isSkip) { continue; }

    RCLCPP_DEBUG_STREAM(get_logger(), magic_enum::enum_name(dir));
    changeDirection(dir);
    lastKey = key;
  }
  changeDirection(DIR::STOP);
  restoreConsoleMode();
} catch(const std::exception& e) {
  changeDirection(DIR::STOP);
  restoreConsoleMode();
  throw Domabot::Exception::BackTrack(e);
}

void CLI::runCLI() try {
  enum class USER_COMMAND : uint8_t {
      BRAKE   = 0
    , STOP    = 1
    , MOVE    = 2
    , UPDATE  = 3
    , SAVE    = 4
    , MODE    = 5
    , DIR     = 6
    , QUIT    = 7
    , GET     = 8
    , DIR_KEY = 9
    , ENBL    = 10
  };

  bool isTerminate = false;
  while (rclcpp::ok() && !isTerminate) {
    try {
      const auto command =  UserInteraction::proposeOptions<USER_COMMAND>(
          "Select command to send controller.\nAvailable commands:"
        , {
            { "b" , "Brake"             , USER_COMMAND::BRAKE   }
          , { "st", "Stop"              , USER_COMMAND::STOP    }
          , { "mv", "Move to target"    , USER_COMMAND::MOVE    }
          , { "g" , "Get data"          , USER_COMMAND::GET     }
          , { "u" , "Update settings"   , USER_COMMAND::UPDATE  }
          , { "sv", "Save settings"     , USER_COMMAND::SAVE    }
          , { "md", "Change mode"       , USER_COMMAND::MODE    }
          , { "d" , "Change dir"        , USER_COMMAND::DIR     }
          , { "q" , "Quit"              , USER_COMMAND::QUIT    }
          , { "k" , "Direct mode by key", USER_COMMAND::DIR_KEY }
          , { "e" , "Enable motors"     , USER_COMMAND::ENBL    }
        });

      switch (command) {
        case USER_COMMAND::BRAKE: {
          callService<DI::srv::Brake>(
              m_clientBrake
            , std::make_shared<DI::srv::Brake::Request>());
          break;
        }
        case USER_COMMAND::STOP: {
          callService<DI::srv::Stop>(
              m_clientStop
            , std::make_shared<DI::srv::Stop::Request>());
          break;
        }
        case USER_COMMAND::MOVE: {
          const auto req = std::make_shared<DI::srv::Move::Request>();

          req->target_position_left = StringTools::stringToNumber<int16_t>(
            UserInteraction::askInput("Enter target position for left stepper: "));
          req->target_position_right = StringTools::stringToNumber<int16_t>(
            UserInteraction::askInput("Enter target position for right stepper: "));

          callService<DI::srv::Move>(m_clientMove, req);
          break;
        }
        case USER_COMMAND::GET: {
          const auto res = callService<DI::srv::GetData>(
              m_clientGetData
            , std::make_shared<DI::srv::GetData::Request>());

          const auto& status = res->status;
          const auto& settings = res->settings;
          const auto& stepperLeft = settings.stepper_left.front();
          const auto& stepperRight = settings.stepper_right.front();

          RCLCPP_INFO_STREAM(get_logger(),
              "Command: " << getEnumItemName<CMD>(res->command.command, "command") << std::endl
            << "Mode: " << getEnumItemName<MODE>(res->mode.mode, "mode") << std::endl
            << "Direction: " << getEnumItemName<DIR>(res->direction.direction, "direction") << std::endl
            << "Status:" << std::endl
            << " - Controller status: " << getEnumItemName<STS>(status.controller.status, "status") << std::endl
            << " - Left stepper:" << std::endl
            << " - - status: " <<  getEnumItemName<STPR_STS>(status.stepper_left.status, "stepper status") << std::endl
            << " - - position: " <<  status.stepper_left.position << std::endl
            << " - Right stepper:" << std::endl
            << " - - status: " <<  getEnumItemName<STPR_STS>(status.stepper_right.status, "stepper status") << std::endl
            << " - - position: " <<  status.stepper_right.position << std::endl
            << "Settings:" << std::endl
            << " - Update rate: " <<  settings.update_rate.front() << std::endl
            << " - Left stepper:" << std::endl
            << " - - target: "  <<  stepperLeft.target.front() << std::endl
            << " - - max speed: "  <<  stepperLeft.max_speed.front() << std::endl
            << " - - max acceleration: "  <<  stepperLeft.max_acceleration.front() << std::endl
            << " - - gear ratio: "  <<  stepperLeft.gear_ratio.front() << std::endl
            << " - - wheel diameter: "  <<  static_cast<unsigned int>(stepperLeft.wheel_diameter.front()) << std::endl
            << " - - is forward: "  <<  static_cast<unsigned int>(stepperLeft.is_forward.front()) << std::endl
            << " - Right stepper:" << std::endl
            << " - - target: "  <<  stepperRight.target.front() << std::endl
            << " - - max speed: "  <<  stepperRight.max_speed.front() << std::endl
            << " - - max acceleration: "  <<  stepperRight.max_acceleration.front() << std::endl
            << " - - gear ratio: "  <<  stepperRight.gear_ratio.front() << std::endl
            << " - - wheel diameter: "  <<  static_cast<unsigned int>(stepperRight.wheel_diameter.front()) << std::endl
            << " - - is forward: "  <<  static_cast<unsigned int>(stepperRight.is_forward.front()) << std::endl
          );
          break;
        }
        case USER_COMMAND::UPDATE: {
          const auto req = std::make_shared<DI::srv::SetSettings::Request>();
          inputSettings(req->settings);
          callService<DI::srv::SetSettings>(m_clientSetSettings, req);
          break;
        }
        case USER_COMMAND::SAVE: {
          DI::msg::ControllerSettings newSettings;
          const bool updated = inputSettings(newSettings);

          const auto req = std::make_shared<DI::srv::SaveSettings::Request>();
          if (updated) {
            req->settings.push_back(newSettings);
          }
          callService<DI::srv::SaveSettings>(m_clientSaveSettings, req);
          break;
        }
        case USER_COMMAND::MODE: {
          enum class USER_MODE: uint8_t {
              TRG  = 0
            , DRCT = 1
            , WRD  = 2
          };

          const USER_MODE mode = UserInteraction::proposeOptions<USER_MODE>(
            "Select mode on change\nAvailable operation mods:"
            , {
                {"t", "Target"   , USER_MODE::TRG  }
              , {"d", "Direction", USER_MODE::DRCT }
              , {"w", "Wired",     USER_MODE::WRD  }
            });

          const auto req = std::make_shared<DI::srv::SetMode::Request>();
          switch (mode) {
            default: { [[fallthrough]]; }
            case USER_MODE::TRG: {
              req->mode.mode = DI::msg::Mode::MODE_TRG;
              break;
            }
            case USER_MODE::DRCT: {
              req->mode.mode = DI::msg::Mode::MODE_DRCT;
              break;
            }
            case USER_MODE::WRD: {
              req->mode.mode = DI::msg::Mode::MODE_WRD;
              break;
            }
          }
          callService<DI::srv::SetMode>(m_clientSetMode, req);
          break;
        }
        case USER_COMMAND::DIR: {
          enum class USER_DIRECTION : uint8_t {
              STOP     = 0
            , FORWARD  = 1
            , RIGHT    = 2
            , BACKWARD = 3
            , LEFT     = 4
            , QUIT     = 6
          };
          bool isQuit = false;
          while (rclcpp::ok() && !isQuit) {
            const USER_DIRECTION direction =
              UserInteraction::proposeOptions<USER_DIRECTION>(
                "Select direction on change\nAvailable directions:"
                  , {
                      {"s", "Stop"   ,  USER_DIRECTION::STOP     }
                    , {"f", "Forward",  USER_DIRECTION::FORWARD  }
                    , {"r", "Right",    USER_DIRECTION::RIGHT    }
                    , {"b", "Backward", USER_DIRECTION::BACKWARD }
                    , {"l", "Left",     USER_DIRECTION::LEFT     }
                    , {"q", "Quit",     USER_DIRECTION::QUIT     }
                  });

            if (USER_DIRECTION::QUIT == direction) {
              break;
            }

            DIR dir = DIR::STOP;
            switch (direction) {
              default: { [[fallthrough]]; }
              case USER_DIRECTION::STOP:     { dir = DIR::STOP;     break; }
              case USER_DIRECTION::FORWARD:  { dir = DIR::FORWARD;  break; }
              case USER_DIRECTION::RIGHT:    { dir = DIR::RIGHT;    break; }
              case USER_DIRECTION::BACKWARD: { dir = DIR::BACKWARD; break; }
              case USER_DIRECTION::LEFT:     { dir = DIR::LEFT;     break; }
            }
            changeDirection(dir);
          }
          break;
        }
        case USER_COMMAND::DIR_KEY: {
          runDirectMode();
          break;
        }
        case USER_COMMAND::ENBL: {
          const auto req = std::make_shared<DI::srv::EnableMotors::Request>();
          while (rclcpp::ok()) {
            const std::string entered = UserInteraction::askInput("Enter enable motors signal value (1/0): ");
            int signal = StringTools::stringToNumber<int>(entered);
            if (0 == signal || 1 == signal) {
              req->enable_motors = 0 != signal;
              break;
            }
          }
          callService<DI::srv::EnableMotors>(m_clientEnableMotors, req);
          break;
        }
        case USER_COMMAND::QUIT: {
          isTerminate = true;
          break;
        }
      }
    } catch(const std::exception& e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    }
  }
} defaultCatch

#undef DI

}  // namespace Domabot
