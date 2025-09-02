/**
 * @file Controller.h
 * @brief Domabot Controller class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CONTROLLER__CONTROLLER_H_
#define DOMABOT_CONTROLLER__CONTROLLER_H_

#include <domabot_controller/Modbus.h>

#include <domabot_common_lib/Exception.h>

#include <domabot_controller_interfaces/msg/status.hpp>
#include <domabot_controller_interfaces/srv/brake.hpp>
#include <domabot_controller_interfaces/srv/enable_motors.hpp>
#include <domabot_controller_interfaces/srv/get_data.hpp>
#include <domabot_controller_interfaces/srv/move.hpp>
#include <domabot_controller_interfaces/srv/save_settings.hpp>
#include <domabot_controller_interfaces/srv/set_direction.hpp>
#include <domabot_controller_interfaces/srv/set_mode.hpp>
#include <domabot_controller_interfaces/srv/set_settings.hpp>
#include <domabot_controller_interfaces/srv/stop.hpp>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <magic_enum/magic_enum.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace Domabot {

/** @brief Class node for communication with micro-controller mega2560. */
class Controller : public rclcpp::Node {
  public:
    using Ptr = std::shared_ptr<Controller>;
    using CnstPtr = std::shared_ptr<const Controller>;

  protected:
    #define DI domabot_controller_interfaces
    static const char m_statusTopicName[];  ///< Status topic name.

    /** @brief Modbus class wrapper, contains connections descriptor. */
    Modbus::Ptr m_modbus = nullptr;

    rclcpp::Publisher<DI::msg::Status>::SharedPtr
      m_pubStatus = nullptr;

    rclcpp::Service<DI::srv::Brake>::SharedPtr m_srvBrake = nullptr;
    rclcpp::Service<DI::srv::EnableMotors>::SharedPtr m_srvEnableMotors = nullptr;
    rclcpp::Service<DI::srv::GetData>::SharedPtr m_srvGetData = nullptr;
    rclcpp::Service<DI::srv::Move>::SharedPtr m_srvMove = nullptr;
    rclcpp::Service<DI::srv::SaveSettings>::SharedPtr m_srvSaveSettings = nullptr;
    rclcpp::Service<DI::srv::SetDirection>::SharedPtr m_srvSetDirection = nullptr;
    rclcpp::Service<DI::srv::SetMode>::SharedPtr m_srvSetMode = nullptr;
    rclcpp::Service<DI::srv::SetSettings>::SharedPtr m_srvSetSettings = nullptr;
    rclcpp::Service<DI::srv::Stop>::SharedPtr m_srvStop = nullptr;

    /** @brief Timer requesting data from controller for status publishing. */
    rclcpp::TimerBase::SharedPtr m_statusTimer = nullptr;

    /** @brief Reentrant call back group for timers. */
    rclcpp::CallbackGroup::SharedPtr m_timerCallbackGroup = nullptr;

    /** @brief Multi-thread call back group for services. */
    rclcpp::CallbackGroup::SharedPtr m_serviceCallbackGroup = nullptr;

    std::atomic_uint16_t m_statusRate{1};  ///< Rate for status publishing.
    std::mutex m_mtxTimer;  ///< Mutex for rate & status timer.

    /** @brief Checks subscribers for status and restart status timer. */
    rclcpp::TimerBase::SharedPtr m_statsTimer = nullptr;

    /** @brief Count subscribers on Status topic. */
    std::atomic_bool m_isStatusSubscriber{0};

    /** @brief Moving flag for blocking another not emergency commands. */
    std::atomic_bool m_isMoving{0};
    MODE m_currentMode = MODE::TRG;  ///< Current robot behavior.
    bool m_isMotorsEnabled = false;  ///< Allow all movement commands.

    /**
     * @brief Check that value in enum.
     * @tparam EnumType Already declared enum for check.
     * @param[in] value Checking value as register value.
     * @param[in] enumName Printable enum's name.
     * @param[in] selfCheck Check for internal use.
     * @return Checked item of EnumType.
     * @throws If items outside enum bounds and this is not self check.
     */
    template<typename EnumType> static EnumType checkEnumItem(
        const uint16_t value
      , const std::string& enumName
      , const bool selfCheck = false
    ) try {
      const auto item = magic_enum::enum_cast<EnumType>(value);
      if (!item.has_value() && !selfCheck) {
        throw Exception::createError("Unknown ", enumName, " item: ", value);
      }
      return item.value();
    } defaultCatch

    /**
     * @brief Check obtained execution command status.
     * @details Useful for obtained execution status from external source and
     * detection protocol errors.
     * @param[in] value Command execution status as register value.
     * @param[in] selfCheck Flag for internal use, disable throwing
     * exceptions.
     * @return Checked status as item of STS enum.
     * @throws If status is not OK.
     */
    STS checkStatus(const uint16_t value, const bool selfCheck = false) const;

    /**
     * @brief Check obtained mode for valid value.
     * @details Useful for obtained mode from external source and
     * detection protocol errors.
     * @param[in] value Mode as register value.
     * @param[in] selfCheck Flag for internal use, disable throwing
     * exceptions for hardware activated modes.
     * @return Checked mode as item of MODE enum.
     * @throws If mode outside MODE enum values, or mode can't be
     * activated by software way.
     */
    static MODE checkMode(const uint16_t value, const bool selfCheck = false);

    /**
     * @brief Check obtained direction for valid value.
     * @details Useful for obtained direction from external source and
     * detection protocol errors.
     * @param[in] value Direction as register value.
     * @return Checked direction as item of DIR enum.
     * @throws If direction outside DIR enum values.
     */
    static DIR checkDirection(const uint16_t value);

    /**
     * @brief Check obtained stepper status for valid value.
     * @details Useful for obtained direction from external source and
     * detection protocol errors.
     * @param[in] value Stepper status as register value.
     * @return Checked stepper status as item of STPR_STS enum.
     * @throws If stepper status outside STPR_STS enum values.
     */
    static STPR_STS checkStepperStatus(const uint16_t value);

    /**
     * @brief Check obtained command for valid value.
     * @details Useful for obtained command from external source and
     * detection protocol errors.
     * @param[in] value Command as register value.
     * @return Checked command as item of CMD enum.
     * @throws If command outside CMD enum values.
     */
    static CMD checkCommand(const uint16_t value);

    /**
     * @brief Validates current mode to mode of requested command.
     * @param[in] cmd Requested command.
     * @param[in] mode Requested mode for requested command.
     * @throws If requested mode not equal current mode.
     */
    void checkAllowedMode(const CMD cmd, const MODE mode) {
      if (mode != m_currentMode) {
        throw Exception::createError(
            "Can't execute command ", magic_enum::enum_name(cmd)
          , " in mode ", magic_enum::enum_name(m_currentMode)
          , "! This command execute only in "
          , magic_enum::enum_name(mode), " mode.");
      }
    }

    /**
     * @brief Write controller settings to Modbus Holding registers.
     * @details Write only filled settings fields.
     * @param[in] settings Controller settings.
     */
    void setSettingsToRegisters(
      const DI::msg::ControllerSettings& settings);

    /**
     * @brief Send & run command on micro-controller via Modbus.
     * @details Performs handshake for command transfer to controller.
     * Wait accept & complete execution from micro-controller side.
     * @param[in] cmd Valid command from CMD.
     * @throws If can't execute command at this time, for non emergency commands.
     * @throws If command invalid.
     * @throws If micro-controller does not accepting or ignoring command.
     */
    void runCommand(const CMD cmd);

    /**
     * @brief Template for run command via service call.
     * @details Calls to runCommand method and fill positive response fields.
     * @tparam Service Processed service.
     * @param[in] res Pointer to not filled service response.
     * @param[in] command Command to execution.
     * @throws Nothing. Catch all exceptions and print it.
     */
    template <typename Service> void processRequestCommand(
      std::shared_ptr<typename Service::Response> res,
      const CMD command
    ) try {
      runCommand(command);
      res->response_data.is_success = true;
      res->response_data.error_message = "";
    } catch (const std::exception& e) {
      res->response_data.is_success = false;
      const auto msg = Exception::BackTrack(e).what();
      res->response_data.error_message = msg;
      RCLCPP_ERROR_STREAM(get_logger(), msg);
    }

    /**
     * @brief Template for exception processing raised within service call.
     * @details Catch all exceptions, print it and fill negative response fields.
     * @tparam Service Processed service.
     * @param[in] res Pointer to not filled service response.
     * @param[in] e Catched exception.
     */
    template <typename Service> void processExceptionCommand(
      std::shared_ptr<typename Service::Response> res,
      const std::exception& e
    ) noexcept {
      res->response_data.is_success = false;
      const auto msg = Exception::BackTrack(e).what();
      res->response_data.error_message = msg;
      RCLCPP_ERROR_STREAM(get_logger(), msg);
    }

    /**
     * @brief Template to fill container by external obtained value.
     * @tparam Container Register values container (unordered_map)
     * @param[in] holdingRegs Ref to container with holding register values.
     * @param[in] address Register addres, must be valid value from REG_HLD enum.
     * @param[in] container Ref to external container with value. Container
     * may be empty.
     * @param[in] name Printable register's name for error message.
     * @param[in] validateAboveZero Flag enables above zero validation for register
     * value.
     * @throws if validateAboveZero active and register value equal or less zero.
     */
    template <typename Container> void setRegister(
        Modbus::HoldingRegistersValues& holdingRegs
      , const REG_HLD address
      , const Container& container
      , const std::string& name = ""
      , const bool validateAboveZero = false
    ) try {
      if (!container.empty()) {
        if (validateAboveZero && 0 == container.front()) {
          throw Exception::createError(name, " must be above zero!");
        }
        holdingRegs.emplace(address, container.front());
      }
    } defaultCatch

    /**
     * @brief Restart status timer by new rate.
     * @details Should calls on every rate setting change. If rate not changed
     * do nothing.
     * @param[in] rate New rate value for status update in status topic.
     * @throws If rate is zero, because zero division.
     */
    void restartStatusTimer(const uint16_t rate);

    /** @brief Performs emergency stops robot's wheels. */
    void brakeSrvCallback(
        const std::shared_ptr<DI::srv::Brake::Request> req
      , std::shared_ptr<DI::srv::Brake::Response> res);

    /** @brief Switch enable motors signal. */
    void enableMotorsSrvCallback(
        const std::shared_ptr<DI::srv::EnableMotors::Request> req
      , std::shared_ptr<DI::srv::EnableMotors::Response> res);

    /** @brief Service for obtain all status & settings data from
     * micro-controller. */
    void getDataSrvCallback(
        const std::shared_ptr<DI::srv::GetData::Request> req
      , std::shared_ptr<DI::srv::GetData::Response> res);

    /** @brief Send target wheels positions and move command to it. */
    void moveSrvCallback(
        const std::shared_ptr<DI::srv::Move::Request> req
      , std::shared_ptr<DI::srv::Move::Response> res);

    /** @brief Send settings to micro-controller, apply & save them to EEPROM. */
    void saveSettingsSrvCallback(
        const std::shared_ptr<DI::srv::SaveSettings::Request> req
      , std::shared_ptr<DI::srv::SaveSettings::Response> res);

    /** @brief Change robot direction move in Direction (DIR) mode. */
    void setDirectionSrvCallback(
        const std::shared_ptr<DI::srv::SetDirection::Request> req
      , std::shared_ptr<DI::srv::SetDirection::Response> res);

    /** @brief Change robot operation mode. */
    void setModeSrvCallback(
        const std::shared_ptr<DI::srv::SetMode::Request> req
      , std::shared_ptr<DI::srv::SetMode::Response> res);

    /** @brief Send settings to micro-controller, apply them without saving. */
    void setSettingsSrvCallback(
        const std::shared_ptr<DI::srv::SetSettings::Request> req
      , std::shared_ptr<DI::srv::SetSettings::Response> res);

    /** @brief Performs slow down stop wheels. */
    void stopSrvCallback(
        const std::shared_ptr<DI::srv::Stop::Request> req
      , std::shared_ptr<DI::srv::Stop::Response> res);

    /** @brief Request status from micro-controller and publish it to
     * status topic. */
    void statusTimerCallback();

    /** @brief Check status topic subscriber and stop/starts status timer for its. */
    void statsTimerCallback();

    #undef DI

  public:
    Controller();

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() = default;
};  // Controller

}  // namespace Domabot

#endif  // DOMABOT_CONTROLLER__CONTROLLER_H_
