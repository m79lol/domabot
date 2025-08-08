/**
 * @file Controller.h
 * @brief Domabot Controller class header file.
*/
#ifndef Domabot_Controller_h
#define Domabot_Controller_h

#include <domabot_controller/Modbus.h>

#include <domabot_common_lib/Exception.h>

#include <domabot_interfaces/msg/status.hpp>
#include <domabot_interfaces/srv/brake.hpp>
#include <domabot_interfaces/srv/get_data.hpp>
#include <domabot_interfaces/srv/move.hpp>
#include <domabot_interfaces/srv/save_settings.hpp>
#include <domabot_interfaces/srv/set_direction.hpp>
#include <domabot_interfaces/srv/set_mode.hpp>
#include <domabot_interfaces/srv/set_settings.hpp>
#include <domabot_interfaces/srv/stop.hpp>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace Domabot {

/**
 * @brief Class node for communication with micro-controller mega2560.
 */
class Controller : public rclcpp::Node {
  public:
    using Ptr = std::shared_ptr<Controller>;
    using CnstPtr = std::shared_ptr<const Controller>;
  protected:
    static const std::string m_statusTopicName; ///< Status topic name.
    Modbus::Ptr m_modbus = nullptr; ///< Modbus class wrapper, contains connections descriptor.

    rclcpp::Publisher<domabot_interfaces::msg::Status>::SharedPtr m_pubStatus = nullptr;

    rclcpp::Service<domabot_interfaces::srv::Brake>::SharedPtr m_srvBrake = nullptr;
    rclcpp::Service<domabot_interfaces::srv::GetData>::SharedPtr m_srvGetData = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Move>::SharedPtr m_srvMove = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SaveSettings>::SharedPtr m_srvSaveSettings = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetDirection>::SharedPtr m_srvSetDirection = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetMode>::SharedPtr m_srvSetMode = nullptr;
    rclcpp::Service<domabot_interfaces::srv::SetSettings>::SharedPtr m_srvSetSettings = nullptr;
    rclcpp::Service<domabot_interfaces::srv::Stop>::SharedPtr m_srvStop = nullptr;

    rclcpp::TimerBase::SharedPtr m_statusTimer = nullptr; ///< Timer requesting data from controller for status publishing.
    rclcpp::CallbackGroup::SharedPtr m_timerCallbackGroup = nullptr; ///< Reentrant call back group for timers.
    uint16_t m_statusRate = 1; ///< Rate for status publishing.
    std::mutex m_mtxTimer; ///< Mutex for rate & status timer.

    rclcpp::TimerBase::SharedPtr m_statsTimer = nullptr; ///< Checks subscribers for status and restart status timer.
    std::atomic_size_t m_cntStatusSubscriber{0}; ///< Count subscribers on Status topic.

    bool m_isCommandExecuting = false; ///< Command executing flag for blocking another not emergency commands.
    MODE m_currentMode = MODE::TRG; ///< Current robot behavior.

    /**
     * @brief Get printable name of command.
     *
     * @param[in] command Robot command from CMD enum.
     * @return String command name.
     */
    static const std::string& getCommandName(const CMD command);

    /**
     * @brief Get printable name of command execution status.
     *
     * @param[in] status Status from STS enum.
     * @return String status name.
     */
    static const std::string& getStatusName(const STS status);

    /**
     * @brief Get printable name of robot operation mode.
     *
     * @param[in] mode Mode from MODE enum.
     * @return String mode name.
     */
    static const std::string& getModeName(const MODE mode);

    /**
     * @brief Check obtained execution command status.
     * @details Useful for obtained execution status from external source and
     * detection protocol errors.
     *
     * @param[in] status Command execution status.
     * @param[in] selfCheck Flag for internal use, disable throwing
     * exceptions.
     * @throws If status is not OK.
     */
    void checkStatus(const STS status, const bool selfCheck = false) const;

    /**
     * @brief Check obtained mode for valid value.
     * @details Useful for obtained mode from external source and
     * detection protocol errors.
     *
     * @param[in] mode Mode from MODE enum.
     * @param[in] selfCheck Flag for internal use, disable throwing
     * exceptions for hardware activated modes.
     * @throws If mode outside MODE enum values, or mode can't be
     * activated by software way.
     */
    static void checkMode(const MODE mode, const bool selfCheck = false);

    /**
     * @brief Check obtained direction for valid value.
     * @details Useful for obtained direction from external source and
     * detection protocol errors.
     *
     * @param[in] direction Mode from DIR enum.
     * @throws If direction outside DIR enum values.
     */
    static void checkDirection(const DIR direction);

    /**
     * @brief Check obtained stepper status for valid value.
     * @details Useful for obtained direction from external source and
     * detection protocol errors.
     *
     * @param[in] stepperStatus Stepper status from STPR_STS enum.
     * @throws If stepper status outside STPR_STS enum values.
     */
    static void checkStepperStatus(const STPR_STS stepperStatus);

    /**
     * @brief Check obtained command for valid value.
     * @details Useful for obtained command from external source and
     * detection protocol errors.
     *
     * @param[in] command Command from CMD enum.
     * @throws If command outside CMD enum values.
     */
    static void checkCommand(const CMD command);

    /**
     * @brief Write controller settings to Modbus Holding registers.
     * @details Write only filled settings fields.
     *
     * @param[in] settings Controller settings.
     */
    void setSettingsToRegisters(
      const domabot_interfaces::msg::ControllerSettings& settings);

    /**
     * @brief Send & run command on micro-controller via Modbus.
     * @details Performs handshake for command transfer to controller.
     * Wait accept & complete execution from micro-controller side.
     *
     * @param[in] cmd Valid command from CMD.
     * @throws If can't execute command at this time, for non emergency commands.
     * @throws If command invalid.
     * @throws If micro-controller does not accepting or ignoring command.
     */
    void runCommand(const CMD cmd);

    /**
     * @brief Template for run command via service call.
     * @details Calls to runCommand method and fill positive response fields.
     *
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
     *
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
     * @brief Template to fill register values container by external obtained value.
     *
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
     * @brief Change status timer state on specify rate and status subscribers count.
     * @details Calls on every status subscribers count or rate setting change. If
     * there is not subscribers, then disable timer. If rate not changed for enabled
     * timer, do nothing.
     *
     * @param[in] rate Rate for status update in status topic.
     * @throws if rate is zero, because zero division.
     */
    void restartStatusTimer(const uint16_t rate);

    /** @brief Performs emergency stops robot's wheels. */
    void brakeSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Brake::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Brake::Response> res);

    /** @brief Service for obtain all status & settings data from micro-controller. */
    void getDataSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::GetData::Request> req
      , std::shared_ptr<domabot_interfaces::srv::GetData::Response> res);

    /** @brief Send target wheels positions and move command to it. */
    void moveSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Move::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Move::Response> res);

    /** @brief Send settings to micro-controller, apply & save them to EEPROM. */
    void saveSettingsSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SaveSettings::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SaveSettings::Response> res);

    /** @brief Change robot direction move in Direction (DIR) mode. */
    void setDirectionSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetDirection::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetDirection::Response> res);

    /** @brief Change robot operation mode. */
    void setModeSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetMode::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetMode::Response> res);

    /** @brief Send settings to micro-controller, apply them without saving. */
    void setSettingsSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::SetSettings::Request> req
      , std::shared_ptr<domabot_interfaces::srv::SetSettings::Response> res);

    /** @brief Performs slow down stop wheels. */
    void stopSrvCallback(
        const std::shared_ptr<domabot_interfaces::srv::Stop::Request> req
      , std::shared_ptr<domabot_interfaces::srv::Stop::Response> res);

    /** @brief Request status from micro-controller and publish it to status topic. */
    void statusTimerCallback();

    /** @brief Check status topic subscriber and stop/starts status timer for its. */
    void statsTimerCallback();

  public:
    Controller();

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() = default;

}; // Controller

} // Domabot

#endif // Domabot_Controller_h