
#include <domabot_cli/CLI.h>
#include <domabot_cli/StringTools.h>
#include <domabot_cli/UserInteraction.h>

#include <domabot_common_lib/RosService.h>

namespace Domabot {

CLI::CLI() try : Node("domabot_cli") {
  m_subStatus = create_subscription<domabot_interfaces::msg::Status>(
      "status", 1, std::bind(&CLI::statusCallback, this, std::placeholders::_1));

  m_clientBrake        = create_client<domabot_interfaces::srv::Brake>("brake");
  m_clientGetData      = create_client<domabot_interfaces::srv::GetData>("get_data");
  m_clientMove         = create_client<domabot_interfaces::srv::Move>("move");
  m_clientSaveSettings = create_client<domabot_interfaces::srv::SaveSettings>("save_settings");
  m_clientSetDirection = create_client<domabot_interfaces::srv::SetDirection>("set_direction");
  m_clientSetMode      = create_client<domabot_interfaces::srv::SetMode>("set_mode");
  m_clientSetSettings  = create_client<domabot_interfaces::srv::SetSettings>("set_settings");
  m_clientStop         = create_client<domabot_interfaces::srv::Stop>("stop");

} defaultCatch

void CLI::statusCallback(const domabot_interfaces::msg::Status& msg) try {
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
  };

  bool isTerminate = false;
  while (rclcpp::ok() && !isTerminate) {

    const auto command =  UserInteraction::proposeOptions<USER_COMMAND>("Available commands:", {
        {"br", "Brake"          , USER_COMMAND::BRAKE }
      , {"st", "Stop"           , USER_COMMAND::STOP  }
      , {"mv", "Move to target" , USER_COMMAND::MOVE  }
      , {"up", "Update settings", USER_COMMAND::UPDATE}
      , {"sv", "Save settings"  , USER_COMMAND::SAVE  }
      , {"cm", "Change mode"    , USER_COMMAND::MODE  }
      , {"cd", "Change dir"     , USER_COMMAND::DIR   }
      , {"q" , "Quit"           , USER_COMMAND::QUIT  }
    });

    switch (command) {
      case USER_COMMAND::BRAKE: {
        RosService::callAndVerifyService<domabot_interfaces::srv::Brake>(
            shared_from_this()
          , get_logger().get_child("service")
          , m_clientBrake
          , std::make_shared<domabot_interfaces::srv::Brake::Request>()
        );
        break;
      }
      case USER_COMMAND::STOP: {
        RosService::callAndVerifyService<domabot_interfaces::srv::Stop>(
            shared_from_this()
          , get_logger().get_child("service")
          , m_clientStop
          , std::make_shared<domabot_interfaces::srv::Stop::Request>()
        );
        break;
      }
      case USER_COMMAND::MOVE: {
        const auto req = std::make_shared<domabot_interfaces::srv::Move::Request>();

        req->target_position_left = StringTools::stringToNumber<int16_t>(
          UserInteraction::askInput("Enter target position for left stepper: "));
        req->target_position_right = StringTools::stringToNumber<int16_t>(
          UserInteraction::askInput("Enter target position for right stepper: "));

        RosService::callAndVerifyService<domabot_interfaces::srv::Move>(
            shared_from_this()
          , get_logger().get_child("service")
          , m_clientMove
          , req
        );
        break;
      }
      case USER_COMMAND::UPDATE: {
        break;
      }
      case USER_COMMAND::SAVE: {
        break;
      }
      case USER_COMMAND::MODE: {
        break;
      }
      case USER_COMMAND::DIR: {
        break;
      }
      case USER_COMMAND::QUIT: {
        isTerminate = true;
        break;
      }
    }
  }

} defaultCatch

} // Domabot