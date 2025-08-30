/**
 * @file CLI.cpp
 * @brief Domabot CLI node class source file.
 * @copyright Copyright 2025 m79lol
*/
#include <domabot_cli/CLI.h>

#include <domabot_firmware/firmware_data_types.h>

namespace Domabot {

#define DI domabot_interfaces

CLI::CLI() try : Node("domabot_cli") {
  m_subStatus = create_subscription<DI::msg::Status>(
      "status", 1, std::bind(&CLI::statusCallback, this, std::placeholders::_1));

  m_clientBrake        = create_client<DI::srv::Brake>("brake");
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

void CLI::runCLI() try {
  enum class USER_COMMAND : uint8_t {
      BRAKE  = 0
    , STOP   = 1
    , MOVE   = 2
    , UPDATE = 3
    , SAVE   = 4
    , MODE   = 5
    , DIR    = 6
    , QUIT   = 7
    , GET    = 8
  };

  bool isTerminate = false;
  while (rclcpp::ok() && !isTerminate) {
    try {
      const auto command =  UserInteraction::proposeOptions<USER_COMMAND>(
          "Select command to send controller.\nAvailable commands:"
        , {
            {"br", "Brake"          , USER_COMMAND::BRAKE }
          , {"st", "Stop"           , USER_COMMAND::STOP  }
          , {"mv", "Move to target" , USER_COMMAND::MOVE  }
          , {"gt", "Get data"       , USER_COMMAND::GET   }
          , {"up", "Update settings", USER_COMMAND::UPDATE}
          , {"sv", "Save settings"  , USER_COMMAND::SAVE  }
          , {"cm", "Change mode"    , USER_COMMAND::MODE  }
          , {"cd", "Change dir"     , USER_COMMAND::DIR   }
          , {"q" , "Quit"           , USER_COMMAND::QUIT  }
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

            const auto req = std::make_shared<DI::srv::SetDirection::Request>();
            switch (direction) {
              default: { [[fallthrough]]; }
              case USER_DIRECTION::STOP: {
                req->direction.direction = DI::msg::Direction::DIR_STOP;
                break;
              }
              case USER_DIRECTION::FORWARD: {
                req->direction.direction = DI::msg::Direction::DIR_FORWARD;
                break;
              }
              case USER_DIRECTION::RIGHT: {
                req->direction.direction = DI::msg::Direction::DIR_RIGHT;
                break;
              }
              case USER_DIRECTION::BACKWARD: {
                req->direction.direction = DI::msg::Direction::DIR_BACKWARD;
                break;
              }
              case USER_DIRECTION::LEFT: {
                req->direction.direction = DI::msg::Direction::DIR_LEFT;
                break;
              }
            }
            callService<DI::srv::SetDirection>(m_clientSetDirection, req);
          }
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
