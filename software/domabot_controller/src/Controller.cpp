
#include <domabot_controller/Controller.h>
#include <domabot_controller/Exception.h>

namespace Domabot {

const std::string Controller::ParamNames::m_baudRate = "baud_rate";
const std::string Controller::ParamNames::m_controllerPath = "controller_path";
const std::string Controller::ParamNames::m_dataBits = "data_bits";
const std::string Controller::ParamNames::m_parity   = "parity";
const std::string Controller::ParamNames::m_stopBits = "stop_bits";

Controller::Controller(
  const rclcpp::Node::SharedPtr node
) try : m_node(node) {

  m_node->declare_parameter(ParamNames::m_baudRate, 9600);
  m_node->declare_parameter(ParamNames::m_controllerPath, "/dev/ttyUSB0");
  m_node->declare_parameter(ParamNames::m_dataBits, 8);
  m_node->declare_parameter(ParamNames::m_parity, 'N');
  m_node->declare_parameter(ParamNames::m_stopBits, 1);

  m_cntx = modbus_new_rtu(
    m_node->get_parameter(ParamNames::m_controllerPath).as_string().c_str(),
    m_node->get_parameter(ParamNames::m_baudRate).as_int(),
    m_node->get_parameter(ParamNames::m_parity).as_string().at(0),
    m_node->get_parameter(ParamNames::m_dataBits).as_int(),
    m_node->get_parameter(ParamNames::m_stopBits).as_int()
  );

  if (NULL == ctx) {
    throw Exception::create("Unable to create the libmodbus context!");
  }

  m_statusPublisher =
    m_node->create_publisher<domabot_interfaces::msg::ControllerStatus>(
      "controller_status", 10);

  using namespace std::chrono_literals;
  m_statusTimer = m_node->create_wall_timer(
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

void Controller::statusTimerCallback() try {
  auto msg = domabot_interfaces::msg::ControllerStatus();

  m_statusPublisher->publish(msg);
} catch (const std::exception& e) {
  RCLCPP_INFO_STREAM(m_node->get_logger(), e.what());
}

} // Domabot