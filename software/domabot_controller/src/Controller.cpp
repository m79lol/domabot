
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
    throw Exception::createError("Set modbus slave address error: ", modbus_strerror(errno));
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "...ok");

  // TODO load rate from controller & set publisher

  m_statusPublisher =
    create_publisher<domabot_interfaces::msg::ControllerStatus>(
      "controller_status", 10);

  using namespace std::chrono_literals;
  m_statusTimer = create_wall_timer(
      500ms, std::bind(&Controller::statusTimerCallback, this));
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
          throw Exception::createError("Could not establish link. Modbus error: ", modbus_strerror(errno));
        }
        m_isConnected = true;
        isNewConnect = true;
        RCLCPP_DEBUG_STREAM(get_logger(), "Reconnected.");
      }
      if (!operation(m_cntx)) {
        if (m_isConnected && !isNewConnect) {
          RCLCPP_DEBUG_STREAM(get_logger(), "Modbus operation failed. Modbus error: " << modbus_strerror(errno));
          RCLCPP_DEBUG_STREAM(get_logger(), "Trying to re-establish link...");
          modbus_close(m_cntx);
          m_isConnected = false;
        }
        throw Exception::createError("Modbus operation failed after link was re-established. Modbus error: ", modbus_strerror(errno));
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

void Controller::statusTimerCallback() try {
  auto msg = domabot_interfaces::msg::ControllerStatus();

  m_statusPublisher->publish(msg);
} catch (const std::exception& e) {
  RCLCPP_INFO_STREAM(get_logger(), e.what());
}

} // Domabot